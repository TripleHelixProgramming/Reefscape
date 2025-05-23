package frc.robot.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.CoralRollerConstants;

public class CoralRoller extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(CoralRollerConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkLimitSwitch coralSensor = motor.getForwardLimitSwitch();
  public final Trigger hasCoral = new Trigger(() -> coralSensor.isPressed());
  public final Trigger isRolling = new Trigger(() -> Math.abs(getRollerVelocity()) > 1);

  public CoralRoller() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultCurrentLimit)
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

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Roller/Velocity", encoder.getVelocity());
    // SmartDashboard.putNumber("Coral Roller/Applied Duty Cycle", motor.getAppliedOutput());
    // SmartDashboard.putNumber("Coral Roller/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Coral Loaded", hasCoral.getAsBoolean());
    SmartDashboard.putBoolean("Coral isRolling", isRolling.getAsBoolean());
  }

  private void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  public double getRollerVelocity() {
    return encoder.getVelocity();
  }

  public Command stop() {
    return this.startEnd(() -> motor.set(0.0), () -> {}).withName("Stop");
  }

  public Command intake() {
    return this.run(() -> setVoltage(CoralRollerConstants.kIntakeVoltage)).withName("Intake");
  }

  public Command outtakeToL1() {
    return this.run(() -> setVoltage(CoralRollerConstants.kOuttakeToL1Voltage))
        .withName("Outtake to L1");
  }

  public Command outtakeToL2() {
    return this.run(() -> setVoltage(CoralRollerConstants.kOuttakeToL2Voltage))
        .withName("Outtake to L2");
  }

  public Command outtakeToL3() {
    return this.run(() -> setVoltage(CoralRollerConstants.kOuttakeToL3Voltage))
        .withName("Outtake to L3");
  }

  public Command outtakeToL4() {
    return this.run(() -> setVoltage(CoralRollerConstants.kOuttakeToL4Voltage))
        .withName("Outtake to L4");
  }

  public Command jiggle() {
    return Commands.sequence(intake().withTimeout(0.3), outtakeToL1().withTimeout(0.05))
        .withName("Jiggle");
  }
}
