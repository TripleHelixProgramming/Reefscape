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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerGains;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;

public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;

  private final SparkMaxConfig globalConfig = new SparkMaxConfig();
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private final DigitalInput lowerLimitSwitch;
  private final DigitalInput upperLimitSwitch;

  private final EventLoop loop = new EventLoop();

  /**
   * private final ProfiledPIDController m_positionController = new ProfiledPIDController(
   * IntakeConstants.kPositionP, IntakeConstants.kPositionI, IntakeConstants.kPositionD,
   * IntakeConstants.kConstraints);
   */
  private Elevator() {
    leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorPort, MotorType.kBrushless);
    followerMotor = new SparkMax(ElevatorConstants.kFollowerMotorPort, MotorType.kBrushless);

    globalConfig
      .voltageCompensation(RobotConstants.kNominalVoltage)
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);

    leaderConfig
      .apply(globalConfig);

    followerConfig
      .apply(globalConfig)
      .follow(leaderMotor);

    leaderConfig.closedLoop.maxMotion
      .maxAcceleration(ElevatorConstants.kMaxAcceleration)
      .maxVelocity(ElevatorConstants.kMaxVelocity);

    leaderConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(ElevatorControllerGains.kP)
      .i(ElevatorControllerGains.kI)
      .d(ElevatorControllerGains.kD)
      // .iZone()
      // .velocityFF(ElevatorConstants.kFF)
      .outputRange(-1, 1);

    leaderConfig.encoder
      .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller = leaderMotor.getClosedLoopController();
    encoder = leaderMotor.getEncoder();

    lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitSwitchPort);
    upperLimitSwitch = new DigitalInput(ElevatorConstants.upperLimitSwitchPort);

    BooleanEvent atLowerLimit = new BooleanEvent(loop, lowerLimitSwitch::get);
    atLowerLimit.rising().ifHigh(() -> resetEncoder());
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());

    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      encoder.setPosition(0);
    }

    SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Lower Limit Switch", upperLimitSwitch.get());
  }

  private void resetEncoder() {
    encoder.setPosition(0);
  }

  private void setPosition(ElevatorPosition position) {
    controller.setReference(position.height, ControlType.kPosition);
  }

  public Command createSetPositionCommand(ElevatorPosition position) {
    return new InstantCommand(() -> this.setPosition(position));
  }

  public Command createStopCommand() {
    return this.runOnce(
        () -> {
          leaderMotor.set(0.0);
        });
  }
}
