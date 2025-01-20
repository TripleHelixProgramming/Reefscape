package frc.robot.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

public enum SwerveModule {
  FrontLeft(
      DriveConstants.MotorControllers.kFrontLeftDriveMotorPort,
      DriveConstants.MotorControllers.kFrontLeftTurningMotorPort,
      DriveConstants.AbsoluteEncoders.kFrontLeftTurningEncoderPort),
  FrontRight(
      DriveConstants.MotorControllers.kFrontRightDriveMotorPort,
      DriveConstants.MotorControllers.kFrontRightTurningMotorPort,
      DriveConstants.AbsoluteEncoders.kFrontRightTurningEncoderPort),
  RearLeft(
      DriveConstants.MotorControllers.kRearLeftDriveMotorPort,
      DriveConstants.MotorControllers.kRearLeftTurningMotorPort,
      DriveConstants.AbsoluteEncoders.kRearLeftTurningEncoderPort),
  RearRight(
      DriveConstants.MotorControllers.kRearRightDriveMotorPort,
      DriveConstants.MotorControllers.kRearRightTurningMotorPort,
      DriveConstants.AbsoluteEncoders.kRearRightTurningEncoderPort);

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final SparkMaxConfig m_defaultMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_driveMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_turningMotorConfig = new SparkMaxConfig();

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;

  private final SparkClosedLoopController m_driveController;
  private final SparkClosedLoopController m_turningController;

  private final CANcoder m_turningAbsEncoder;
  private final CANcoderConfiguration m_turningAbsEncoderConfig;

  private SwerveModule(
      int driveMotorChannel, int turningMotorChannel, int turningAbsoluteEncoderChannel) {

    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_defaultMotorConfig
      .voltageCompensation(RobotConstants.kNominalVoltage)
      .inverted(false);
    
    m_driveMotorConfig
      .apply(m_defaultMotorConfig)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    
    m_turningMotorConfig
      .apply(m_defaultMotorConfig)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    m_driveMotorConfig.closedLoop
      .p(DriveControllerGains.kP)
      .i(DriveControllerGains.kI)
      .d(DriveControllerGains.kD)
      // .iZone();
      .velocityFF(DriveControllerGains.kFF);
      // .outputRange();

    m_turningMotorConfig.closedLoop
      .p(TurningControllerGains.kP)
      .i(TurningControllerGains.kI)
      .d(TurningControllerGains.kD)
      // .iZone();
      // .outputRange();
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(-0.5, 0.5);

    m_driveMotorConfig.encoder
      .positionConversionFactor(ModuleConstants.kDrivePositionConversionFactor)
      .velocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);

    m_turningMotorConfig.encoder
      .positionConversionFactor(ModuleConstants.kTurnPositionConversionFactor);

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
        name() + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
        magnetOffsetFromCANCoder);
    double magnetOffsetFromPreferences =
        Preferences.getDouble(
            name() + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
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
        name() + DriveConstants.AbsoluteEncoders.kAbsEncoderMagnetOffsetKey,
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
    return name();
  }

  public double getDriveMotorCurrent() {
    return m_driveMotor.getOutputCurrent();
  }
}
