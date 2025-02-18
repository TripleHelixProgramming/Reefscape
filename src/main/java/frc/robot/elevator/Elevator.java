package frc.robot.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorController;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor =
      new SparkMax(ElevatorConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax followerMotor =
      new SparkMax(ElevatorConstants.kFollowerMotorPort, MotorType.kBrushless);
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(ElevatorConstants.lowerLimitSwitchPort);

  private final SparkMaxConfig globalConfig = new SparkMaxConfig();
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final ProfiledPIDController controller =
      new ProfiledPIDController(ElevatorController.kP, 0.0, 0.0, ElevatorController.kConstraints);

  private final EventLoop loop = new EventLoop();

  private ElevatorState targetState = ElevatorState.Unknown;

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

    leaderConfig.encoder
        .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

    leaderConfig.softLimit
        .reverseSoftLimit(ElevatorState.Floor.height)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(ElevatorState.Max.height)
        .forwardSoftLimitEnabled(true);
    // spotless:on

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    BooleanEvent atLowerLimit = new BooleanEvent(loop, () -> !lowerLimitSwitch.get());
    atLowerLimit.rising().ifHigh(() -> resetEncoder());

    controller.setTolerance(ElevatorConstants.kAllowableHeightError);
    // controller.setIZone();
    // controller.setIntegratorRange();
    resetPositionController();
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Elevator Height", encoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", encoder.getVelocity());

    SmartDashboard.putString("Elevator Target State", getTargetState().name());
    SmartDashboard.putNumber("Elevator Target Height", controller.getGoal().position);

    if (SmartDashboard.getBoolean("Elevator Reset Encoder", false)) {
      SmartDashboard.putBoolean("Elevator Reset Encoder", false);
      resetEncoder();
    }

    SmartDashboard.putNumber("Elevator Lead Applied Duty Cycle", leaderMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "Elevator Follower Applied Duty Cycle", followerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Lead Current", leaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Follower Current", followerMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Elevator Lower Limit Switch", !lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Elevator At Target Height", controller.atGoal());
  }

  public double getProportionOfMaxHeight() {
    return encoder.getPosition() / ElevatorState.Max.height;
  }

  public void resetPositionController() {
    controller.reset(encoder.getPosition());
    controller.setGoal(encoder.getPosition());
  }

  public ElevatorState getTargetState() {
    return this.targetState;
  }

  private void resetEncoder() {
    encoder.setPosition(ElevatorState.Reset.height);
  }

  public Command createSetPositionCommand(ElevatorState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          controller.setGoal(targetState.height);
        },
        // execute
        () -> {
          leaderMotor.set(controller.calculate(encoder.getPosition()));
        },
        // end
        interrupted -> {},
        // isFinished
        () -> controller.atGoal(),
        // requirements
        this);
  }

  private Boolean isInRange(double height) {
    if (height < ElevatorState.Min.height) return false;
    if (height > ElevatorState.Max.height) return false;
    return true;
  }

  public Command createJoystickControlCommand(XboxController gamepad) {
    return this.run(
        () -> {
          double targetPosition = controller.getGoal().position;

          double joystickInput = MathUtil.applyDeadband(-gamepad.getLeftY(), 0.05);
          targetPosition +=
              joystickInput
                  * ElevatorConstants.kFineVelocityInchesPerSecond
                  * RobotConstants.kPeriod;

          if (isInRange(targetPosition)) controller.setGoal(targetPosition);
          leaderMotor.set(controller.calculate(encoder.getPosition()));
        });
  }
}
