package frc.robot.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerPositionGains;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerVelocityGains;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.RobotConstants;

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

  private ElevatorPosition currentPosition;
  private ElevatorPosition nextPosition;

  /**
   * private final ProfiledPIDController m_positionController = new ProfiledPIDController(
   * IntakeConstants.kPositionP, IntakeConstants.kPositionI, IntakeConstants.kPositionD,
   * IntakeConstants.kConstraints);
   */
  public Elevator() {
    leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorPort, MotorType.kBrushless);
    followerMotor = new SparkMax(ElevatorConstants.kFollowerMotorPort, MotorType.kBrushless);

    // spotless:off
    globalConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kCurrentLimit);

    leaderConfig
        .apply(globalConfig)
        .inverted(false);

    followerConfig
        .apply(globalConfig)
        .inverted(true)
        .follow(leaderMotor);

    leaderConfig.closedLoop.maxMotion
        .maxAcceleration(ElevatorConstants.kMaxAcceleration)
        .maxVelocity(ElevatorConstants.kMaxVelocity);

    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ElevatorControllerPositionGains.kP , ClosedLoopSlot.kSlot0)
        .i(ElevatorControllerPositionGains.kI, ClosedLoopSlot.kSlot0)
        .d(ElevatorControllerPositionGains.kD, ClosedLoopSlot.kSlot0)
        // .iZone()
        // .velocityFF(ElevatorConstants.kFF)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .p(ElevatorControllerVelocityGains.kP, ClosedLoopSlot.kSlot1)
        .i(ElevatorControllerVelocityGains.kI, ClosedLoopSlot.kSlot1)
        .d(ElevatorControllerVelocityGains.kD, ClosedLoopSlot.kSlot1)
        // .velocityFF(ElevatorConstants.kFF, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    leaderConfig.encoder
        .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);
    // spotless:on

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

    currentPosition = ElevatorPosition.Floor;
    nextPosition = currentPosition;
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Elevator Height", encoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", encoder.getVelocity());

    SmartDashboard.putString("Elevator Position Current", getCurrentPosition().name());
    SmartDashboard.putString("Elevator Position Next", getNextPosition().name());

    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      resetEncoder();
    }

    SmartDashboard.putBoolean("Lower Limit Switch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Lower Limit Switch", upperLimitSwitch.get());
  }

  private ElevatorPosition getCurrentPosition() {
    return this.currentPosition;
  }

  private ElevatorPosition getNextPosition() {
    return this.nextPosition;
  }

  private void incrementState(int increment) {
    ElevatorPosition[] positions = ElevatorPosition.values();
    int nextIndex = (currentPosition.ordinal() + increment) % positions.length;
    nextPosition = positions[nextIndex];
  }

  private void resetEncoder() {
    encoder.setPosition(0);
  }

  private void setPosition(ElevatorPosition position) {
    currentPosition = position;
    controller.setReference(position.height, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void setVelocity(double targetVelocity) {
    controller.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public ElevatorPosition getElevatorPosition() {
    return currentPosition;
  }

  public Command createSetPositionCommand(ElevatorPosition position) {
    return new InstantCommand(() -> this.setPosition(position));
  }

  public Command createGoToNextPositionCommand() {
    return new InstantCommand(() -> this.setPosition(nextPosition));
  }

  public Command createStopCommand() {
    return this.runOnce(
        () -> {
          leaderMotor.set(0.0);
        });
  }

  public Command createJoystickControlCommand(XboxController controller, double factor) {
    return this.run(
        () -> {
          this.setVelocity(controller.getLeftY() * factor);
        });
  }

  public Command createNextStateCommand() {
    return this.runOnce(() -> incrementState(1));
  }

  public Command createPreviousStateCommand() {
    return this.runOnce(() -> incrementState(-1));
  }
}
