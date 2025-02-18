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
import frc.robot.Constants.AlgaeWristConstants;
import frc.robot.Constants.RobotConstants;

public class AlgaeWrist extends SubsystemBase {

  private final SparkMax wristMotor =
      new SparkMax(AlgaeWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig wristConfig = new SparkMaxConfig();
  private final SparkAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController wristController = wristMotor.getClosedLoopController();

  public AlgaeWrist() {
    // spotless:off
    wristConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEOCurrentLimit)
        .inverted(false);
    
    wristConfig.absoluteEncoder
        .positionConversionFactor(AlgaeWristConstants.kPositionConversionFactor)
        .zeroOffset(AlgaeWristConstants.kPositionOffset)
        .inverted(true);
    
    wristConfig.closedLoop
        .p(AlgaeWristConstants.kPositionP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // spotless:on

    wristMotor.configure(
        wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Rotation Position", wristEncoder.getPosition());
  }

  private void setRotationPosition(double position) {
    wristController.setReference(position, ControlType.kPosition);
  }

  public Command createSetRotationPositionCommand(double position) {
    return this.runOnce(() -> setRotationPosition(position));
  }
}
