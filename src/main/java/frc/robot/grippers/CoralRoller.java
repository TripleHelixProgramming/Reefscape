package frc.robot.grippers;

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
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.RobotConstants;
import java.util.function.BooleanSupplier;

public class CoralRoller extends SubsystemBase {

  private final SparkMax rollerMotor =
      new SparkMax(CoralIntakeConstants.kRollerMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
  private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
  private final SparkClosedLoopController rollerController = rollerMotor.getClosedLoopController();
  private final DigitalInput coralSensor = new DigitalInput(CoralIntakeConstants.kCoralSensorPort);

  public CoralRoller() {
    // spotless:off
    rollerConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    rollerConfig.closedLoop
        .p(CoralIntakeConstants.kVelocityP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

    rollerConfig.encoder
        .velocityConversionFactor(CoralIntakeConstants.kVelocityConversionFactor)
        .positionConversionFactor(CoralIntakeConstants.kPositionConversionFactor);
    // spotless:on

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Intake Velocity", rollerEncoder.getVelocity());
    SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
  }

  private void resetIntakeEncoder() {
    rollerEncoder.setPosition(0.0);
  }

  private void setIntakeVelocity(double velocity) {
    rollerController.setReference(velocity, ControlType.kVelocity);
  }

  public Trigger hasCoral = new Trigger(() -> coralSensor.get());

  public BooleanSupplier hasCoralPiece() {
    return () -> (coralSensor.get() == true);
  }

  public Command createStopIntakeCommand() {
    return this.runOnce(() -> setIntakeVelocity(0));
  }

  public Command createSetIntakeVelocityCommand(double velocity) {
    return this.run(() -> setIntakeVelocity(velocity));
  }
}
