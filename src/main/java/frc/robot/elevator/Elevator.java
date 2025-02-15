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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerPositionGains;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerVelocityGains;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(ElevatorConstants.kFollowerMotorPort, MotorType.kBrushless);

  private final SparkMaxConfig globalConfig = new SparkMaxConfig();
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkClosedLoopController controller = leaderMotor.getClosedLoopController();

  private final DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitSwitchPort);

  private final EventLoop loop = new EventLoop();

  private ElevatorPosition targetPosition;
  private ElevatorPosition nextPosition;

  /**
   * private final ProfiledPIDController m_positionController = new ProfiledPIDController(
   * IntakeConstants.kPositionP, IntakeConstants.kPositionI, IntakeConstants.kPositionD,
   * IntakeConstants.kConstraints);
   */
  public Elevator() {
    // spotless:off
    globalConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEOCurrentLimit)
        .inverted(false);

    leaderConfig
        .apply(globalConfig);

    followerConfig
        .apply(globalConfig)
        .follow(leaderMotor, true);

    leaderConfig.closedLoop.maxMotion
        .maxVelocity(ElevatorConstants.kMaxVelocityRPM)
        .maxAcceleration(ElevatorConstants.kMaxAccelerationRPMPerSecond);

    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        .p(ElevatorControllerPositionGains.kP , ClosedLoopSlot.kSlot0)
        .i(ElevatorControllerPositionGains.kI, ClosedLoopSlot.kSlot0)
        .d(ElevatorControllerPositionGains.kD, ClosedLoopSlot.kSlot0)
        // .iZone()
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        
        .p(ElevatorControllerVelocityGains.kP, ClosedLoopSlot.kSlot1)
        .i(ElevatorControllerVelocityGains.kI, ClosedLoopSlot.kSlot1)
        .d(ElevatorControllerVelocityGains.kD, ClosedLoopSlot.kSlot1)
        // .velocityFF(ElevatorConstants.kFF, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    leaderConfig.encoder
        .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

    leaderConfig.softLimit
        .reverseSoftLimit(ElevatorPosition.Floor.height)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(ElevatorPosition.Max.height)
        .forwardSoftLimitEnabled(true);
    // spotless:on

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    BooleanEvent atLowerLimit = new BooleanEvent(loop, () -> !lowerLimitSwitch.get());
    atLowerLimit.rising().ifHigh(() -> resetEncoder());

    targetPosition = ElevatorPosition.Floor;
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Elevator Height", encoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", encoder.getVelocity());

    SmartDashboard.putString("Elevator Target Position Name", getTargetPosition().name());
    SmartDashboard.putNumber("Elevator Target Position Height", getTargetPosition().height);

    if (SmartDashboard.getBoolean("Elevator Reset Encoder", false)) {
      SmartDashboard.putBoolean("Elevator Reset Encoder", false);
      resetEncoder();
    }

    SmartDashboard.putNumber("Elevator Lead Applied Duty Cycle", leaderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Follower Applied Duty Cycle", followerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Lead Current", leaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Follower Elevator Current", followerMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Elevator Lower Limit Switch", !lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Elevator isAtHeight", isAtTargetHeight());
  }

  public ElevatorPosition getTargetPosition() {
    return this.targetPosition;
  }

  private ElevatorPosition getNextPosition() {
    return this.nextPosition;
  }

  private void incrementState(int increment) {
    ElevatorPosition[] positions = ElevatorPosition.values();
    int nextIndex = (targetPosition.ordinal() + increment) % positions.length;
    nextPosition = positions[nextIndex];
  }

  public void resetEncoder() {
    encoder.setPosition(ElevatorPosition.Reset.height);
  }

  private void setPosition(ElevatorPosition targetPosition) {
    controller.setReference(targetPosition.height, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  private void setVelocity(double targetVelocity) {
    controller.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  private Boolean isAtTargetHeight() {
    return (Math.abs(encoder.getPosition() - targetPosition.height) < ElevatorConstants.kAllowableHeightError);
  }

  public Command createSetPositionCommand(ElevatorPosition position) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetPosition = position;
          setPosition(position);
        },
        // execute
        () -> {},
        // end
        interrupted -> {},
        // isFinished
        this::isAtTargetHeight,
        // requirements
        this);
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
          this.setVelocity(MathUtil.applyDeadband(-controller.getLeftY(), 0.02) * factor);
        });
  }

  public Command createNextStateCommand() {
    return this.runOnce(() -> incrementState(1));
  }

  public Command createPreviousStateCommand() {
    return this.runOnce(() -> incrementState(-1));
  }
}
