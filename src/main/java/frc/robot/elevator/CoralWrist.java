package frc.robot.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants.NEO550Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.CoralWristConstants;
import frc.robot.elevator.ElevatorConstants.CoralWristConstants.CoralWristState;
import java.util.function.Supplier;

public class CoralWrist extends SubsystemBase {

  private final SparkMax motor = new SparkMax(CoralWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(
          CoralWristConstants.kP,
          CoralWristConstants.kI,
          CoralWristConstants.kD,
          CoralWristConstants.kConstraints);
  private final ArmFeedforward feedforward =
      new ArmFeedforward(CoralWristConstants.kS, CoralWristConstants.kG, CoralWristConstants.kV);

  private CoralWristState targetState = CoralWristState.Initial;

  public CoralWrist() {
    // spotless:off
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(NEO550Constants.kDefaultCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    
    config.absoluteEncoder
        .inverted(false) // TODO: determine if has any effect
        .zeroCentered(false)
        .zeroOffset(CoralWristConstants.kZeroOffset.in(Rotations) )
        .positionConversionFactor(CoralWristConstants.kPositionConversionFactor.in(Radians))
        .velocityConversionFactor(CoralWristConstants.kVelocityConversionFactor.in(RadiansPerSecond));

    config.signals
        .primaryEncoderVelocityPeriodMs(100)
        .outputCurrentPeriodMs(100);

    config.softLimit
        .reverseSoftLimit(CoralWristState.Min.angle.in(Radians))
        .reverseSoftLimitEnabled(true)
        // .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(CoralWristState.Max.angle.in(Radians))
        .forwardSoftLimitEnabled(true);
        // .forwardSoftLimitEnabled(false);
    // spotless:on

    motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    feedback.setTolerance(CoralWristConstants.kAllowableError.in(Radians));
    feedback.setIZone(CoralWristConstants.kIZone.in(Radians));
    feedback.enableContinuousInput(0, 2.0 * Math.PI); // TODO: determine if has any effect
    // controller.setIntegratorRange();

    setDefaultCommand(remainAtCurrentAngle());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Wrist/Current Angle Degrees", getCurrentAngle().in(Degrees));
    // SmartDashboard.putNumber("Coral Wrist/Current Angle Radians", encoder.getPosition());
    // SmartDashboard.putNumber("Coral Wrist/Current Angular Velocity RPS", encoder.getVelocity());
    SmartDashboard.putString("Coral Wrist/Target State", getTargetState().name());
    SmartDashboard.putNumber("Coral Wrist/Setpoint Angle Degrees", getSetpointAngle().in(Degrees));
    // SmartDashboard.putNumber("Coral Wrist/Setpoint Angle Radians",
    // feedback.getSetpoint().position);
    // SmartDashboard.putNumber(
    //     "Coral Wrist/Setpoint Angular Velocity RPS", feedback.getSetpoint().velocity);
    // SmartDashboard.putNumber("Coral Wrist/Applied Duty Cycle", motor.getAppliedOutput());
    // SmartDashboard.putNumber("Coral Wrist/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Coral Wrist/At Goal", feedback.atGoal());
    SmartDashboard.putData("Coral Wrist/Controller", feedback);
  }

  private Angle getCurrentAngle() {
    return Radians.of(encoder.getPosition());
  }

  private Angle getSetpointAngle() {
    return Radians.of(feedback.getSetpoint().position);
  }

  public void resetController() {
    feedback.reset(encoder.getPosition(), encoder.getVelocity());
  }

  public void control() {
    double currentPosition = encoder.getPosition();
    double offsetPosition =
        currentPosition + CoralWristConstants.kCenterOfGravityOffset.in(Radians);
    double currentVelocitySetpoint = feedback.getSetpoint().velocity;
    motor.setVoltage(
        feedforward.calculate(offsetPosition, currentVelocitySetpoint)
            + feedback.calculate(currentPosition));
  }

  public CoralWristState getTargetState() {
    return this.targetState;
  }

  public Boolean inState(CoralWristState state) {
    return this.targetState.equals(state);
  }

  public Command remainAtCurrentAngle() {
    return setAngle(() -> getCurrentAngle()).withName("Maintain current angle");
  }

  public Command setAngle(CoralWristState state) {
    return setAngle(() -> state.angle)
        .beforeStarting(new InstantCommand(() -> targetState = state))
        .withName("Set angle to " + state.toString());
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return new FunctionalCommand(
        // initialize
        () -> feedback.setGoal(angleSupplier.get().in(Radians)),
        // execute
        () -> control(),
        // end
        interrupted -> {},
        // isFinished
        () -> false,
        // requirements
        this);
  }

  public Command joystickVoltageControl(XboxController gamepad) {
    return this.run(
        () -> {
          double joystickInput = 2.0 * MathUtil.applyDeadband(-gamepad.getLeftY(), 0.05);
          motor.setVoltage(joystickInput);
        });
  }
}
