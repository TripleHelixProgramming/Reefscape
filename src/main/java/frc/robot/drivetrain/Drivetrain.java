package frc.robot.drivetrain;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.RotationControllerGains;
import frc.robot.Constants.AutoConstants.TranslationControllerGains;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import java.util.function.BooleanSupplier;

/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private BooleanSupplier m_fieldRotatedSupplier;

  static LinearVelocity kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static AngularVelocity kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

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

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  private final Field2d m_field = new Field2d();

  public Drivetrain(BooleanSupplier fieldRotatedSupplier) {

    this.m_fieldRotatedSupplier = fieldRotatedSupplier;

    m_gyro.reset();

    SmartDashboard.putData("Field", m_field);

    for (SwerveModule module : modules) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());

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

    SmartDashboard.putNumber("GyroAngle", m_gyro.getRotation2d().getDegrees());
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

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /** Reconfigures all swerve module steering angles using external alignment device */
  public void zeroAbsTurningEncoderOffsets() {
    for (SwerveModule module : modules) {
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
    m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void resetHeading() {
    Pose2d pose =
        m_fieldRotatedSupplier.getAsBoolean()
            ? new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI))
            : new Pose2d(getPose().getTranslation(), new Rotation2d());

    m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
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
}
