package frc.robot.grippers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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

public class CoralIntake extends SubsystemBase {

  private final SparkMax rollerMotor =
      new SparkMax(CoralIntakeConstants.kRollerMotorPort, MotorType.kBrushless);
  private final SparkMax wristMotor =
      new SparkMax(CoralIntakeConstants.kWristMotorPort, MotorType.kBrushless);

  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
  private final SparkMaxConfig wristConfig = new SparkMaxConfig();

  private final RelativeEncoder rollerEncoder;
  private final SparkAbsoluteEncoder wristEncoder;

  private final SparkClosedLoopController rollerController;
  private final SparkClosedLoopController wristController;

  private final DigitalInput coralSensor = new DigitalInput(CoralIntakeConstants.kCoralSensorPort);

  public CoralIntake() {
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

    wristConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);
    
    wristConfig.closedLoop
        .p(CoralIntakeConstants.kPositionP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    wristConfig.absoluteEncoder
        .positionConversionFactor(CoralIntakeConstants.kCoralWristPositionConversionFactor)
        .zeroOffset(CoralIntakeConstants.kCoralWristPositionOffset)
        .inverted(false);
    // spotless:on

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    wristMotor.configure(
        wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rollerEncoder = rollerMotor.getEncoder();
    wristEncoder = wristMotor.getAbsoluteEncoder();

    rollerController = rollerMotor.getClosedLoopController();
    wristController = wristMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Rotation Position", wristEncoder.getPosition());
    SmartDashboard.putNumber("Coral Intake Velocity", rollerEncoder.getVelocity());

    SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
  }

  private void resetIntakeEncoder() {
    rollerEncoder.setPosition(0.0);
  }

  private void getRotationPosition() {
    wristEncoder.getPosition();
  }

  private void setRotationPosition(double position) {
    wristController.setReference(position, ControlType.kPosition);
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

  public Command createSetRotationPositionCommand(double position) {
    return this.runOnce(() -> setRotationPosition(position));
  }

  public Command createSetIntakeVelocityCommand(double velocity) {
    return this.run(() -> setIntakeVelocity(velocity));
  }
}
