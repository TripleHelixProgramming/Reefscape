package frc.robot.grippers;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralWristConstants;
import frc.robot.Constants.RobotConstants;

public class CoralWrist extends SubsystemBase {

  private final SparkMax wristMotor =
      new SparkMax(CoralWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig wristConfig = new SparkMaxConfig();
  private final SparkAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController wristController = wristMotor.getClosedLoopController();

  public CoralWrist() {
    // spotless:off
    wristConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);
    
    wristConfig.closedLoop
        .p(CoralWristConstants.kPositionP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    wristConfig.absoluteEncoder
        .positionConversionFactor(CoralWristConstants.kPositionConversionFactor)
        .zeroOffset(CoralWristConstants.kPositionOffset)
        .inverted(false);
    // spotless:on

    wristMotor.configure(
        wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Rotation Position", wristEncoder.getPosition());
  }

  private void setRotationPosition(double position) {
    wristController.setReference(position, ControlType.kPosition);
  }

  public Command createSetRotationPositionCommand(double position) {
    return this.runOnce(() -> setRotationPosition(position));
  }
}
