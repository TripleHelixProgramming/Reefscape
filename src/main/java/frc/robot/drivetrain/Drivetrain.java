package frc.robot.drivetrain;

import choreo.trajectory.SwerveSample;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.RotationControllerGains;
import frc.robot.Constants.AutoConstants.TranslationControllerGains;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private BooleanSupplier m_fieldRotatedSupplier;

  static LinearVelocity kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static AngularVelocity kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

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

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics, canandgyro.getRotation2d(), getSwerveModulePositions());

  // private final Field2d m_field = new Field2d();
  private StructPublisher<Pose2d> m_publisher =
      NetworkTableInstance.getDefault().getStructTopic("Odometry", Pose2d.struct).publish();

  public Drivetrain(BooleanSupplier fieldRotatedSupplier) {

    this.m_fieldRotatedSupplier = fieldRotatedSupplier;

    canandgyro.setYaw(0);

    // SmartDashboard.putData("Field", m_field);

    for (SwerveModule module : SwerveModule.values()) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();
    }

    CanandEventLoop.getInstance();
  }

  @Override
  public void periodic() {
    updateOdometry();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    m_publisher.set(m_odometry.getPoseMeters());

    for (SwerveModule module : SwerveModule.values()) {
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

    SmartDashboard.putNumber("GyroAngle", canandgyro.getRotation2d().getDegrees());
  }

  /**
   * @param chassisSpeeds Robot-relative chassis speeds (x, y, theta)
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setChassisSpeeds(chassisSpeeds, DriveConstants.kDriveKinematics);
  }

  /**
   * @param chassisSpeeds Robot-relative chassis speeds (x, y, theta)
   * @param kinematics Kinematics of the robot chassis
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, SwerveDriveKinematics kinematics) {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    for (SwerveModule module : SwerveModule.values()) {
      module.setDesiredState(swerveModuleStates[module.ordinal()]);
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(canandgyro.getRotation2d(), getSwerveModulePositions());
  }

  /** Reconfigures all swerve module steering angles using external alignment device */
  public void zeroAbsTurningEncoderOffsets() {
    for (SwerveModule module : SwerveModule.values()) {
      module.zeroAbsTurningEncoderOffset();
    }
  }

  /**
   * @return The direction of the robot pose
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * @return The robot pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * @param pose The robot pose
   */
  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(canandgyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void resetHeading() {
    Pose2d pose =
        m_fieldRotatedSupplier.getAsBoolean()
            ? new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI))
            : new Pose2d(getPose().getTranslation(), new Rotation2d());

    m_odometry.resetPosition(canandgyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  /**
   * @return Array of swerve module positions
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return Arrays.stream(SwerveModule.values())
        .map(module -> module.getPosition())
        .toArray(SwerveModulePosition[]::new);
  }

  /**
   * @return Array of swerve module states
   */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(SwerveModule.values())
        .map(module -> module.getState())
        .toArray(SwerveModuleState[]::new);
  }

  /**
   * @return The current robot-relative chassis speeds (x, y, theta)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
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
}
