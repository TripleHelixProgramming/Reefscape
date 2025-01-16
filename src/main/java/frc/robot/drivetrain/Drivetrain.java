package frc.robot.drivetrain;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.RotationControllerGains;
import frc.robot.Constants.AutoConstants.TranslationControllerGains;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import java.util.function.BooleanSupplier;

/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private BooleanSupplier m_fieldRotatedSupplier;

  static LinearVelocity kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static AngularVelocity kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

  // The robot pose estimator for tracking swerve odometry and applying vision corrections.
  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          "FrontLeft",
          DriveConstants.MotorControllers.kFrontLeftDriveMotorPort,
          DriveConstants.MotorControllers.kFrontLeftTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kFrontLeftTurningEncoderPort);
  private final SwerveModule m_frontRight =
      new SwerveModule(
          "FrontRight",
          DriveConstants.MotorControllers.kFrontRightDriveMotorPort,
          DriveConstants.MotorControllers.kFrontRightTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kFrontRightTurningEncoderPort);
  private final SwerveModule m_rearLeft =
      new SwerveModule(
          "RearLeft",
          DriveConstants.MotorControllers.kRearLeftDriveMotorPort,
          DriveConstants.MotorControllers.kRearLeftTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kRearLeftTurningEncoderPort);
  private final SwerveModule m_rearRight =
      new SwerveModule(
          "RearRight",
          DriveConstants.MotorControllers.kRearRightDriveMotorPort,
          DriveConstants.MotorControllers.kRearRightTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kRearRightTurningEncoderPort);

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

  private final PIDController xController =
      new PIDController(
          TranslationControllerGains.kP,
          TranslationControllerGains.kI,
          TranslationControllerGains.kD);
  private final PIDController yController =
      new PIDController(
          TranslationControllerGains.kP,
          TranslationControllerGains.kI,
          TranslationControllerGains.kD);
  private final PIDController thetaController =
      new PIDController(
          RotationControllerGains.kP, RotationControllerGains.kI, RotationControllerGains.kD);

  private final Canandgyro canandgyro = new Canandgyro(0);
  private Rotation2d headingOffset = new Rotation2d();

  private StructPublisher<Pose2d> m_visionPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Vision", Pose2d.struct).publish();

  public Drivetrain(BooleanSupplier fieldRotatedSupplier) {

    this.m_fieldRotatedSupplier = fieldRotatedSupplier;

    canandgyro.setYaw(0);

    for (SwerveModule module : modules) {
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
  }

  @Override
  public void periodic() {
    poseEstimator.update(canandgyro.getRotation2d(), getSwerveModulePositions());
    m_visionPosePublisher.set(poseEstimator.getEstimatedPosition());

    for (SwerveModule module : modules) {
      SmartDashboard.putNumber(
          module.getName() + "RelativeTurningPosition",
          module.getRelativeTurningPosition().getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "AbsoluteTurningPosition",
          module.getAbsTurningPosition(0.0).getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "RelativeDrivePosition", module.getRelativeDrivePosition());

      SmartDashboard.putNumber(
          module.getName() + "AbsoluteMagnetOffset",
          module.getAbsTurningEncoderOffset().getDegrees());

      SmartDashboard.putNumber(module.getName() + "OutputCurrent", module.getDriveMotorCurrent());
    }

    SmartDashboard.putNumber("Offset Heading", getOffsetHeading().getDegrees());
  }

  /**
   * @param chassisSpeeds Robot-relative chassis speeds (x, y, theta)
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // uses kinematics type to determine robot center
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, SwerveDriveKinematics kinematicsType) {
    var swerveModuleStates =
        kinematicsType.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Reconfigures all swerve module steering angles using external alignment device */
  public void zeroAbsTurningEncoderOffsets() {
    for (SwerveModule module : modules) {
      module.zeroAbsTurningEncoderOffset();
    }
  }

  public Rotation2d getOffsetHeading() {
    return poseEstimator.getEstimatedPosition().getRotation().minus(headingOffset);
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

  public void resetHeadingOffset() {
    headingOffset = poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * @return Array of swerve module positions
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] bill = {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
    return bill;
  }

  /**
   * @return Array of swerve module states
   */
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
    }

    return states;
  }

  /**
   * @return The current robot-relative chassis speeds (x, y, theta)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * @param curPose The robot's current pose
   * @param sample A sample of the trajectory being followed
   */
  public void choreoController(Pose2d curPose, SwerveSample sample) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(curPose.getX(), sample.x) + sample.vx,
                yController.calculate(curPose.getY(), sample.y) + sample.vy,
                thetaController.calculate(curPose.getRotation().getRadians(), sample.heading)
                    + sample.omega),
            curPose.getRotation());
    this.setChassisSpeeds(speeds);
  }

  public BooleanSupplier fieldRotatedSupplier() {
    return this.m_fieldRotatedSupplier;
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
}
