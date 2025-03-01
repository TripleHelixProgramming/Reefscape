package frc.robot.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.CoralRollerConstants;

public class CoralRoller extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(CoralRollerConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkLimitSwitch coralSensor = motor.getForwardLimitSwitch();

  public CoralRoller() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(true);

    config.limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);

    config.signals
        .absoluteEncoderPositionPeriodMs(100)
        .outputCurrentPeriodMs(100);

    config.encoder
        .velocityConversionFactor(CoralRollerConstants.kVelocityConversionFactor)
        .positionConversionFactor(CoralRollerConstants.kPositionConversionFactor);
    // spotless:on

    motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    setDefaultCommand(createStopCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Roller/Velocity", encoder.getVelocity());
    // SmartDashboard.putNumber("Coral Roller/Applied Duty Cycle", motor.getAppliedOutput());
    // SmartDashboard.putNumber("Coral Roller/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Coral Sensor", coralSensor.isPressed());
  }

  private void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  public Trigger hasCoral = new Trigger(() -> coralSensor.isPressed());

  public Command createStopCommand() {
    return this.startEnd(() -> motor.set(0.0), () -> {});
  }

  public Command createIntakeCommand() {
    return this.run(() -> setVoltage(CoralRollerConstants.kIntakeVoltage));
  }

  public Command createOuttakeCommand() {
    return this.run(() -> setVoltage(CoralRollerConstants.kOuttakeVoltage));
  }
}
