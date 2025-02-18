package frc.robot.elevator;

import static edu.wpi.first.units.Units.InchesPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final DigitalInput coralSensor = new DigitalInput(CoralRollerConstants.kCoralSensorPort);

  public CoralRoller() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    config.closedLoop
        .p(CoralRollerConstants.kP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

    config.encoder
        .velocityConversionFactor(CoralRollerConstants.kVelocityConversionFactor)
        .positionConversionFactor(CoralRollerConstants.kPositionConversionFactor);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    setDefaultCommand(createStopCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Roller Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Coral Roller Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Coral Roller Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
  }

  private void setVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }

  public Trigger hasCoral = new Trigger(() -> coralSensor.get());

  public Command createStopCommand() {
    return this.startEnd(() -> motor.set(0.0), () -> {});
  }

  public Command createIntakeCommand() {
    return this.startEnd(
        () -> setVelocity(CoralRollerConstants.kIntakeSpeed.in(InchesPerSecond)), () -> {});
  }

  public Command createOuttakeCommand() {
    return this.startEnd(
        () -> setVelocity(CoralRollerConstants.kOuttakeSpeed.in(InchesPerSecond)), () -> {});
  }
}
