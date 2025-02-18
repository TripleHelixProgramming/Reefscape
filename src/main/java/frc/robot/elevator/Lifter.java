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
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.LifterConstants.LifterController;
import frc.robot.Constants.LifterConstants.LifterState;
import frc.robot.Constants.RobotConstants;

public class Lifter extends SubsystemBase {

  private final SparkMax leaderMotor =
      new SparkMax(LifterConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax followerMotor =
      new SparkMax(LifterConstants.kFollowerMotorPort, MotorType.kBrushless);
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(LifterConstants.lowerLimitSwitchPort);

  private final SparkMaxConfig globalConfig = new SparkMaxConfig();
  private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final ProfiledPIDController controller =
      new ProfiledPIDController(LifterController.kP, 0.0, 0.0, LifterController.kConstraints);

  private final EventLoop loop = new EventLoop();

  private LifterState targetState = LifterState.Unknown;

  public Lifter() {
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
        .positionConversionFactor(LifterConstants.kPositionConversionFactor)
        .velocityConversionFactor(LifterConstants.kVelocityConversionFactor);

    leaderConfig.softLimit
        .reverseSoftLimit(LifterState.Floor.height)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(LifterState.Max.height)
        .forwardSoftLimitEnabled(true);
    // spotless:on

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    BooleanEvent atLowerLimit = new BooleanEvent(loop, () -> !lowerLimitSwitch.get());
    atLowerLimit.rising().ifHigh(() -> resetEncoder());

    controller.setTolerance(LifterConstants.kAllowableHeightError);
    // controller.setIZone();
    // controller.setIntegratorRange();
    resetController();
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Lifter Height", encoder.getPosition());
    SmartDashboard.putNumber("Lifter Velocity", encoder.getVelocity());

    SmartDashboard.putString("Lifter Target State", getTargetState().name());
    SmartDashboard.putNumber("Lifter Target Height", controller.getGoal().position);

    if (SmartDashboard.getBoolean("Lifter Reset Encoder", false)) {
      SmartDashboard.putBoolean("Lifter Reset Encoder", false);
      resetEncoder();
    }

    SmartDashboard.putNumber("Lifter Leader Applied Duty Cycle", leaderMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "Lifter Follower Applied Duty Cycle", followerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Lifter Lead Current", leaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Lifter Follower Current", followerMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Lifter Lower Limit Switch", !lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Lifter At Target Height", controller.atGoal());
  }

  public double getProportionOfMaxHeight() {
    return encoder.getPosition() / LifterState.Max.height;
  }

  public void resetController() {
    controller.reset(encoder.getPosition());
    controller.setGoal(encoder.getPosition());
  }

  public void control() {
    leaderMotor.set(controller.calculate(encoder.getPosition()));
  }

  public LifterState getTargetState() {
    return this.targetState;
  }

  private void resetEncoder() {
    encoder.setPosition(LifterState.Reset.height);
  }

  public Command createSetHeightCommand(LifterState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          controller.setGoal(targetState.height);
        },
        // execute
        () -> control(),
        // end
        interrupted -> {},
        // isFinished
        () -> controller.atGoal(),
        // requirements
        this);
  }

  public Command createRemainAtCurrentHeightCommand() {
    return new FunctionalCommand(
        // initialize
        () -> {
          if (targetState == LifterState.Unknown) controller.setGoal(encoder.getPosition());
        },
        // execute
        () -> control(),
        // end
        interrupted -> {},
        // isFinished
        () -> false,
        // requirements
        this);
  }

  private Boolean isInRange(double height) {
    if (height < LifterState.Min.height) return false;
    if (height > LifterState.Max.height) return false;
    return true;
  }

  public Command createJoystickControlCommand(XboxController gamepad) {
    return this.run(
        () -> {
          double targetPosition = controller.getGoal().position;

          double joystickInput = MathUtil.applyDeadband(-gamepad.getLeftY(), 0.05);
          targetPosition +=
              joystickInput * LifterConstants.kFineVelocityInchesPerSecond * RobotConstants.kPeriod;

          if (isInRange(targetPosition)) controller.setGoal(targetPosition);
          control();
        });
  }
}
