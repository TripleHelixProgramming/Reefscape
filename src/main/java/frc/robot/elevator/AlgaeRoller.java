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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.AlgaeRollerConstants;
import java.util.function.BooleanSupplier;

public class AlgaeRoller extends SubsystemBase {

  private final SparkMax leaderMotor =
      new SparkMax(AlgaeRollerConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax followerMotor =
      new SparkMax(AlgaeRollerConstants.kFollowerMotorPort, MotorType.kBrushless);

  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkLimitSwitch algaeSensor = leaderMotor.getForwardLimitSwitch();
  public final BooleanSupplier hasAlgae = () -> algaeSensor.isPressed();
  public final BooleanSupplier isRolling = () -> Math.abs(encoder.getVelocity()) > 1;

  public AlgaeRoller() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    config.limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);

    config.encoder
        .velocityConversionFactor(AlgaeRollerConstants.kVelocityConversionFactor)
        .positionConversionFactor(AlgaeRollerConstants.kPositionConversionFactor);

    config.signals
        .absoluteEncoderPositionPeriodMs(100);

    followerConfig
        .apply(config)
        .follow(leaderMotor, true);
    // spotless:on

    leaderMotor.configureAsync(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configureAsync(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    setDefaultCommand(createStopCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Roller/Velocity", encoder.getVelocity());
    // SmartDashboard.putNumber("Algae Roller/Applied Duty Cycle", leaderMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Algae Roller/Current", leaderMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Algae Loaded", hasAlgae.getAsBoolean());
    SmartDashboard.putBoolean("Algae isRolling", isRolling.getAsBoolean());
  }

  private void setVoltage(Voltage voltage) {
    leaderMotor.setVoltage(voltage);
  }

  public double getRollerVelocity() {
    return encoder.getVelocity();
  }

  public Command createStopCommand() {
    return this.startEnd(() -> leaderMotor.set(0.0), () -> {});
  }

  public Command createIntakeCommand() {
    return this.run(() -> setVoltage(AlgaeRollerConstants.kIntakeVoltage));
  }

  public Command createOuttakeCommand() {
    return this.run(() -> setVoltage(AlgaeRollerConstants.kOuttakeVoltage));
  }

  public Command createHoldAlgaeCommand() {
    return this.run(() -> setVoltage(AlgaeRollerConstants.kHoldVoltage));
  }
}
