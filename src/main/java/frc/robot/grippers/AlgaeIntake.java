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
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.RobotConstants;

public class AlgaeIntake extends SubsystemBase {

  private final SparkMax rollerLeaderMotor =
      new SparkMax(AlgaeIntakeConstants.kRollerLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax rollerFollowerMotor =
      new SparkMax(AlgaeIntakeConstants.kRollerFollowerMotorPort, MotorType.kBrushless);
  private final SparkMax wristMotor =
      new SparkMax(AlgaeIntakeConstants.kWristMotorPort, MotorType.kBrushless);

  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
  private final SparkMaxConfig rollerFollowerConfig = new SparkMaxConfig();
  private final SparkMaxConfig wristConfig = new SparkMaxConfig();
  
  private final RelativeEncoder rollerEncoder;
  private final SparkAbsoluteEncoder wristEncoder;

  private final SparkClosedLoopController rollerController;
  private final SparkClosedLoopController wristController;

  private final DigitalInput algaeSensor = new DigitalInput(AlgaeIntakeConstants.kAlgaeSensorPort);

  public AlgaeIntake() {
    // spotless:off
    rollerConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    rollerConfig.closedLoop
        .p(AlgaeIntakeConstants.kVelocityP)
        .i(AlgaeIntakeConstants.kVelocityI)
        .d(AlgaeIntakeConstants.kVelocityD)
        .outputRange(-1, 1);

    rollerConfig.encoder
        .velocityConversionFactor(AlgaeIntakeConstants.kVelocityConversionFactor)
        .positionConversionFactor(AlgaeIntakeConstants.kPositionConversionFactor);

    rollerFollowerConfig
        .apply(rollerConfig)
        .follow(rollerLeaderMotor);

    wristConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEOCurrentLimit)
        .inverted(false);
    
    wristConfig.absoluteEncoder
        .positionConversionFactor(AlgaeIntakeConstants.kAlgaeWristPositionConversionFactor)
        .inverted(false);
    
    wristConfig.closedLoop
        .p(AlgaeIntakeConstants.kPositionP)
        .i(AlgaeIntakeConstants.kPositionI)
        .d(AlgaeIntakeConstants.kPositionD)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // spotless:on

    rollerLeaderMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerFollowerMotor.configure(
        rollerFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    wristMotor.configure(
        wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rollerEncoder = rollerLeaderMotor.getEncoder();
    wristEncoder = wristMotor.getAbsoluteEncoder();

    rollerController = rollerLeaderMotor.getClosedLoopController();
    wristController = wristMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Rotation Position", wristEncoder.getPosition());
    SmartDashboard.putNumber("Algae Intake Velocity", rollerEncoder.getVelocity());

    SmartDashboard.putBoolean("Algae Sensor", algaeSensor.get());
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

  public Trigger hasCoral = new Trigger(() -> algaeSensor.get());

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
