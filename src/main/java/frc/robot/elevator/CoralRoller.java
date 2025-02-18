package frc.robot.elevator;

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
import frc.robot.Constants.CoralRollerConstants;
import frc.robot.Constants.RobotConstants;
import java.util.function.BooleanSupplier;

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
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Intake Velocity", encoder.getVelocity());
    SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
  }

  private void resetIntakeEncoder() {
    encoder.setPosition(0.0);
  }

  private void setIntakeVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }

  public Trigger hasCoral = new Trigger(() -> coralSensor.get());

  public BooleanSupplier hasCoralPiece() {
    return () -> (coralSensor.get() == true);
  }

  public Command createStopCommand() {
    return this.runOnce(() -> setIntakeVelocity(0));
  }

  public Command createSetIntakeCommand() {
    return this.run(() -> setIntakeVelocity(CoralRollerConstants.kIntakeSpeed));
  }

  public Command createSetOuttakeCommand() {
    return this.run(() -> setIntakeVelocity(CoralRollerConstants.kOuttakeSpeed));
  }
}
