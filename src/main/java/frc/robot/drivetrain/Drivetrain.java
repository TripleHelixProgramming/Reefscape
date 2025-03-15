package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants.RotationControllerGains;
import frc.robot.Constants.AutoConstants.TranslationControllerGains;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private BooleanSupplier fieldRotatedSupplier;
  private Supplier<Dimensionless> elevatorHeightSupplier;

  static LinearVelocity kMaxSpeed = DriveConstants.kMaxTranslationalVelocity;
  static LinearVelocity kMinSpeed = DriveConstants.kMinTranslationVelocity;
  static AngularVelocity kMaxAngularSpeed = DriveConstants.kMaxRotationalVelocity;

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

  // The robot pose estimator for tracking swerve odometry and applying vision corrections.
  private final SwerveDrivePoseEstimator poseEstimator;

  private final PIDController xController =
      new PIDController(TranslationControllerGains.kP, 0.0, 0.0);
  private final PIDController yController =
      new PIDController(TranslationControllerGains.kP, 0.0, 0.0);
  private final PIDController thetaController =
      new PIDController(RotationControllerGains.kP, 0.0, 0.0);

  private final Canandgyro canandgyro = new Canandgyro(0);
  private Rotation2d headingOffset = new Rotation2d();

  private StructPublisher<Pose2d> m_visionPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Vision", Pose2d.struct).publish();

  public Drivetrain(BooleanSupplier fieldRotated, Supplier<Dimensionless> elevatorHeight) {
    this.fieldRotatedSupplier = fieldRotated;
    this.elevatorHeightSupplier = elevatorHeight;

    canandgyro.setYaw(0);

    for (SwerveModule module : SwerveModule.values()) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            canandgyro.getRotation2d(),
            getSwerveModulePositions(),
            new Pose2d(),
            DriveConstants.kStateStdDevs,
            VisionConstants.kMultiTagStdDevs);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    poseEstimator.update(canandgyro.getRotation2d(), getSwerveModulePositions());
    m_visionPosePublisher.set(poseEstimator.getEstimatedPosition());

    for (SwerveModule module : SwerveModule.values()) {
      SmartDashboard.putNumber(
          module.getName() + "/RelativeTurningPosition",
          module.getRelativeTurningPosition().getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "/AbsoluteTurningPosition",
          module.getAbsTurningPosition(0.0).getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "/RelativeDrivePosition", module.getRelativeDrivePosition());

      SmartDashboard.putNumber(
          module.getName() + "/AbsoluteMagnetOffset",
          module.getAbsTurningEncoderOffset().getDegrees());

      SmartDashboard.putNumber(module.getName() + "/OutputCurrent", module.getDriveMotorCurrent());
    }

    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("HeadingOffset", headingOffset.getDegrees());
  }

  public void setFieldRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    Rotation2d offsetHeading = getHeading().minus(getHeadingOffset());
    setRobotRelativeChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, offsetHeading));
  }

  /**
   * @param chassisSpeeds Robot-relative chassis speeds (x, y, theta)
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setRobotRelativeChassisSpeeds(chassisSpeeds, DriveConstants.kDriveKinematics);
  }

  /**
   * @param chassisSpeeds Robot-relative chassis speeds (x, y, theta)
   * @param kinematics Kinematics of the robot chassis
   */
  public void setRobotRelativeChassisSpeeds(
      ChassisSpeeds chassisSpeeds, SwerveDriveKinematics kinematics) {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod.in(Seconds)));
    LinearVelocity speedPenalty = (kMaxSpeed.minus(kMinSpeed)).times(elevatorHeightSupplier.get());
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed.minus(speedPenalty));
    for (SwerveModule module : SwerveModule.values()) {
      module.setDesiredState(swerveModuleStates[module.ordinal()]);
    }
  }

  /** Reconfigures all swerve module steering angles using external alignment device */
  public void zeroAbsTurningEncoderOffsets() {
    for (SwerveModule module : SwerveModule.values()) {
      module.zeroAbsTurningEncoderOffset();
    }
  }

  public void refreshRelativeTurningEncoder() {
    for (SwerveModule module : SwerveModule.values()) {
      module.refreshRelativeTurningEncoder();
    }
  }

  public Rotation2d getHeading() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * @return The robot pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @param pose The robot pose
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(canandgyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void initializeRelativeTurningEncoder() {
    for (SwerveModule module : SwerveModule.values()) {
      module.initializeRelativeTurningEncoder();
    }
  }

  public Rotation2d getHeadingOffset() {
    return headingOffset;
  }

  public Command resetHeadingOffset() {
    return new InstantCommand(() -> this.headingOffset = new Rotation2d())
        .withName("reset heading offset");
  }

  public void setHeadingOffset() {
    headingOffset =
        fieldRotatedSupplier.getAsBoolean()
            ? getHeading().rotateBy(new Rotation2d(Math.PI))
            : getHeading();
  }

  /**
   * @return Array of swerve module positions
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return Arrays.stream(SwerveModule.values())
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * @return Array of swerve module states
   */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(SwerveModule.values())
        .map(SwerveModule::getState)
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * @return The current robot-relative chassis speeds (x, y, theta)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public BooleanSupplier fieldRotatedSupplier() {
    return this.fieldRotatedSupplier;
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /**
   * Reset the estimated pose of the swerve drive on the field.
   *
   * @param pose New robot pose.
   * @param resetSimPose If the simulated robot pose should also be reset. This effectively
   *     teleports the robot and should only be used during the setup of the simulation world.
   */
  public void resetPose(Pose2d pose, boolean resetSimPose) {
    poseEstimator.resetPosition(canandgyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void followTrajectory(SwerveSample sample) {
    Pose2d pose = getPose();

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + thetaController.calculate(pose.getRotation().getRadians(), sample.heading));

    setFieldRelativeChassisSpeeds(speeds);
  }

  public Pose2d getNearestPose() {
    Pose2d closestPose = new Pose2d();
    double minDistance = Double.MAX_VALUE;

    for (Pose2d targetPose : DriveConstants.kReefTargetPoses) {
      double distance = getPose().getTranslation().getDistance(targetPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestPose = targetPose;
      }
    }
    return closestPose;
  }
}
