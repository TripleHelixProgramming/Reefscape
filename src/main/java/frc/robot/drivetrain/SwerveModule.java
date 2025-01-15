package frc.robot.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.DriveControllerGains;
import frc.robot.Constants.ModuleConstants.TurningControllerGains;
import frc.robot.Constants.RobotConstants;

public class SwerveModule {
  public final String moduleName;

  private final SparkFlex m_driveMotor;
  private final SparkMax m_turningMotor;
  private final SparkFlexConfig m_driveMotorConfig = new SparkFlexConfig();
  private final SparkMaxConfig m_turningMotorConfig = new SparkMaxConfig();

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;
  private final EncoderConfig m_driveEncoderConfig = new EncoderConfig();
  private final EncoderConfig m_turningRelativeEncoderConfig = new EncoderConfig();

  private final SparkClosedLoopController m_driveController;
  private final SparkClosedLoopController m_turningController;
  private final ClosedLoopConfig m_driveControllerConfig = new ClosedLoopConfig();
  private final ClosedLoopConfig m_turningControllerConfig = new ClosedLoopConfig();

  private final CANcoder m_turningAbsEncoder;
  private final CANcoderConfiguration m_turningAbsEncoderConfig;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param moduleName Name of the module
   * @param driveMotorChannel CAN ID of the drive motor controller
   * @param turningMotorChannel CAN ID of the turning motor controller
   * @param turningAbsoluteEncoderChannel CAN ID of absolute encoder
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAbsoluteEncoderChannel) {
    moduleName = name;

    m_driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotorConfig.voltageCompensation(RobotConstants.kNominalVoltage);
    m_turningMotorConfig.voltageCompensation(RobotConstants.kNominalVoltage);
    m_driveMotorConfig.inverted(false);
    m_turningMotorConfig.inverted(false);

    m_driveMotorConfig.idleMode(IdleMode.kCoast);
    m_turningMotorConfig.idleMode(IdleMode.kBrake);

    m_driveMotorConfig.smartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    m_turningMotorConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    m_driveControllerConfig.p(DriveControllerGains.kP);
    m_driveControllerConfig.i(DriveControllerGains.kI);
    m_driveControllerConfig.d(DriveControllerGains.kD);
    // m_driveControllerConfig.iZone();
    m_driveControllerConfig.velocityFF(DriveControllerGains.kFF);
    // m_driveControllerConfig.outputRange();
    m_driveMotorConfig.apply(m_driveControllerConfig);

    m_turningControllerConfig.p(TurningControllerGains.kP);
    m_turningControllerConfig.i(TurningControllerGains.kI);
    m_turningControllerConfig.d(TurningControllerGains.kD);
    // m_turningControllerConfig.iZone();
    // m_turningControllerConfig.outputRange();
    m_turningControllerConfig.positionWrappingEnabled(true);
    m_turningControllerConfig.positionWrappingInputRange(-0.5, 0.5);
    m_turningMotorConfig.apply(m_turningControllerConfig);

    m_driveEncoderConfig.positionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
    m_driveEncoderConfig.velocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);
    m_driveMotorConfig.apply(m_driveEncoderConfig);

    m_turningRelativeEncoderConfig.positionConversionFactor(
        ModuleConstants.kTurnPositionConversionFactor);
    m_turningMotorConfig.apply(m_turningRelativeEncoderConfig);

    m_driveMotor.configure(
        m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_turningMotor.configure(
        m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_driveController = m_driveMotor.getClosedLoopController();
    m_turningController = m_turningMotor.getClosedLoopController();

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningRelativeEncoder = m_turningMotor.getEncoder();

    m_turningAbsEncoder = new CANcoder(turningAbsoluteEncoderChannel);
    m_turningAbsEncoderConfig = new CANcoderConfiguration();
    m_turningAbsEncoder.getConfigurator().refresh(m_turningAbsEncoderConfig);
    m_turningAbsEncoder.getAbsolutePosition().setUpdateFrequency(50, 0.5);
  }

  /**
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getRelativeTurningPosition());
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getRelativeDrivePosition(), getRelativeTurningPosition());
  }

  /**
   * @param desiredState The desired state for the module, with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = getRelativeTurningPosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    m_driveController.setReference(
        desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningController.setReference(
        desiredState.angle.getRotations(), SparkMax.ControlType.kPosition);
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  /**
   * @return The relative drive position of the module
   */
  public double getRelativeDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  /**
   * @return The relative turning angle of the module
   */
  public Rotation2d getRelativeTurningPosition() {
    double relativePositionRotations = m_turningRelativeEncoder.getPosition();
    return Rotation2d.fromRotations(MathUtil.inputModulus(relativePositionRotations, -0.5, 0.5));
  }

  /**
   * @param waitPeriod Period to wait for up-to-date status signal value
   * @return The absolute turning angle of the module
   */
  public Rotation2d getAbsTurningPosition(double waitPeriod) {
    Angle absPositonRotations;

    if (waitPeriod > 0.0) {
      absPositonRotations =
          m_turningAbsEncoder.getAbsolutePosition().waitForUpdate(waitPeriod).getValue();
    } else {
      absPositonRotations = m_turningAbsEncoder.getAbsolutePosition().getValue();
    }

    return new Rotation2d(absPositonRotations);
  }

  /**
   * Updates the relative turning encoder to match the absolute measurement of the module turning
   * angle.
   */
  public void initializeRelativeTurningEncoder() {
    m_turningRelativeEncoder.setPosition(getAbsTurningPosition(0.25).getRotations());
  }

  /** Initializes the magnetic offset of the absolute turning encoder */
  public void initializeAbsoluteTurningEncoder() {
    double magnetOffsetFromCANCoder = getAbsTurningEncoderOffset().getRotations();
    Preferences.initDouble(
        moduleName + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
        magnetOffsetFromCANCoder);
    double magnetOffsetFromPreferences =
        Preferences.getDouble(
            moduleName + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
            magnetOffsetFromCANCoder);
    setAbsTurningEncoderOffset(magnetOffsetFromPreferences);
  }

  /**
   * Decrements the offset of the absolute turning encoder to align the wheels with an alignment
   * device. To be performed upon a hardware change (e.g. when a swerve module or absolute turning
   * encoder has been swapped.)
   */
  public void zeroAbsTurningEncoderOffset() {
    m_turningAbsEncoder.getConfigurator().refresh(m_turningAbsEncoderConfig);

    Rotation2d magnetOffset = getAbsTurningEncoderOffset().minus(getAbsTurningPosition(0.25));
    Preferences.setDouble(
        moduleName + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
        magnetOffset.getRotations());
    setAbsTurningEncoderOffset(magnetOffset.getRotations());

    initializeRelativeTurningEncoder();
  }

  /**
   * Sets the magnetic offset of the absolute turning encoder
   *
   * @param offset The magnetic offset in rotations
   */
  public void setAbsTurningEncoderOffset(double offset) {
    m_turningAbsEncoderConfig.MagnetSensor.MagnetOffset = offset;
    m_turningAbsEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    m_turningAbsEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    m_turningAbsEncoder.getConfigurator().apply(m_turningAbsEncoderConfig);
  }

  /**
   * @return The magnet offset of the module's absolute turning encoder
   */
  public Rotation2d getAbsTurningEncoderOffset() {
    return Rotation2d.fromRotations(m_turningAbsEncoderConfig.MagnetSensor.MagnetOffset);
  }

  /**
   * @return The name of the swerve module
   */
  public String getName() {
    return moduleName;
  }

  public double getDriveMotorCurrent() {
    return m_driveMotor.getOutputCurrent();
  }
}