package frc.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConstants.NEOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.LifterConstants;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterController;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterState;
import java.util.function.Supplier;

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

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(
          LifterController.kP,
          LifterController.kI,
          LifterController.kD,
          LifterController.kConstraints);

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(LifterController.kS, LifterController.kG, LifterController.kV);

  private final EventLoop loop = new EventLoop();
  private LifterState targetState = LifterState.Initial;

  public Lifter() {
    // spotless:off
    globalConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEOConstants.kDefaultCurrentLimit)
        .inverted(false);

    leaderConfig
        .apply(globalConfig);

    followerConfig
        .apply(globalConfig)
        .follow(leaderMotor, true);

    leaderConfig.encoder
        .positionConversionFactor(LifterConstants.kPositionConversionFactor.in(Meters))
        .velocityConversionFactor(LifterConstants.kVelocityConversionFactor.in(MetersPerSecond));

    leaderConfig.softLimit
        .reverseSoftLimit(LifterState.Min.height.in(Meters))
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(LifterState.Max.height.in(Meters))
        .forwardSoftLimitEnabled(true);

    leaderConfig.signals
        .primaryEncoderVelocityPeriodMs(100);
    // spotless:on

    leaderMotor.configureAsync(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configureAsync(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    BooleanEvent atLowerLimit = new BooleanEvent(loop, () -> !lowerLimitSwitch.get());
    atLowerLimit.rising().ifHigh(() -> resetEncoder());

    feedback.setTolerance(LifterConstants.kAllowableHeightError.in(Meters));
    feedback.setIZone(LifterController.kIzone.in(Inches));
    // controller.setIntegratorRange();
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Lifter/Height", encoder.getPosition());
    // SmartDashboard.putNumber("Lifter/Velocity", encoder.getVelocity());

    SmartDashboard.putString("Lifter/Target State", getTargetState().name());
    SmartDashboard.putNumber("Lifter/Height Setpoint", feedback.getSetpoint().position);
    SmartDashboard.putNumber("Lifter/Height Goal", feedback.getGoal().position);

    // SmartDashboard.putNumber("Lifter/Leader Applied Duty Cycle", leaderMotor.getAppliedOutput());
    // SmartDashboard.putNumber(
    //     "Lifter/Follower Applied Duty Cycle", followerMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Lifter/Leader Current", leaderMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Lifter/Follower Current", followerMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Lifter/Lower Limit Switch", !lowerLimitSwitch.get());
    SmartDashboard.putBoolean("Lifter/At Target Height", feedback.atGoal());
  }

  public Distance getCurrentHeight() {
    return Meters.of(encoder.getPosition());
  }

  public Dimensionless getProportionOfMaxHeight() {
    return getCurrentHeight().div(LifterState.Max.height);
  }

  public void resetController() {
    feedback.reset(encoder.getPosition());
    feedback.setGoal(encoder.getPosition());
  }

  public void control() {
    double currentPosition = encoder.getPosition();
    double currentVelocitySetpoint = feedback.getSetpoint().velocity;
    leaderMotor.setVoltage(
        feedforward.calculate(currentVelocitySetpoint)
            + feedback.calculate(currentPosition)
            + (LifterController.kG * encoder.getPosition()));
  }

  public LifterState getTargetState() {
    return this.targetState;
  }

  public Boolean inState(LifterState state) {
    return this.targetState.equals(state);
  }

  public Trigger atIntakingHeight = new Trigger(() -> inState(LifterState.CoralIntake));
  public Trigger atProcessorHeight = new Trigger(() -> inState(LifterState.AlgaeProcessor));
  public Trigger atBargeHeight = new Trigger(() -> inState(LifterState.AlgaeBarge));
  public Trigger atFloorIntakingHeight = new Trigger(() -> inState(LifterState.AlgaeIntakeFloor));

  private void resetEncoder() {
    encoder.setPosition(LifterState.EncoderReset.height.in(Meters));
  }

  public Command remainAtCurrentHeight() {
    return setHeight(() -> getCurrentHeight()).withName("Maintain current height");
  }

  public Command setHeight(LifterState state) {
    return setHeight(() -> state.height)
        .beforeStarting(new InstantCommand(() -> targetState = state))
        .withName("Set height to " + state.toString());
  }

  public Command setHeight(Supplier<Distance> heightSupplier) {
    return new FunctionalCommand(
        // initialize
        () -> feedback.setGoal(heightSupplier.get().in(Meters)),
        // execute
        () -> control(),
        // end
        interrupted -> {},
        // isFinished
        () -> feedback.atGoal(),
        // requirements
        this);
  }

  public Command matchHeight() {
    return new InstantCommand(() -> feedback.setGoal(encoder.getPosition()));
  }

  private Boolean isInRange(Distance height) {
    if (height.lt(LifterState.Min.height)) return false;
    if (height.gt(LifterState.Max.height)) return false;
    return true;
  }

  public Command joystickVelocityControl(XboxController gamepad) {
    return new FunctionalCommand(
            // initialize
            () -> {},
            // execute
            () -> {
              Distance targetPosition = Meters.of(feedback.getGoal().position);

              double joystickInput = MathUtil.applyDeadband(-gamepad.getLeftY(), 0.05);
              Distance adder =
                  LifterConstants.kFineVelocity.times(joystickInput).times(RobotConstants.kPeriod);
              targetPosition = targetPosition.plus(adder);

              if (isInRange(targetPosition)) feedback.setGoal(targetPosition.in(Meters));
              control();
            },
            // end
            interrupted -> {},
            // isFinished
            () -> false,
            // requirements
            this)
        .withName("Joystick velocity control");
  }

  public Command joystickVoltageControl(XboxController gamepad) {
    return this.run(
            () -> {
              double joystickInput = MathUtil.applyDeadband(-gamepad.getLeftY(), 0.05);
              leaderMotor.setVoltage(joystickInput * 2.0);
            })
        .withName("Joystick voltage control");
  }
}
