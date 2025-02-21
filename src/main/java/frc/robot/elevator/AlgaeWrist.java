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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.AlgaeWristConstants;
import frc.robot.elevator.ElevatorConstants.AlgaeWristConstants.AlgaeWristState;

public class AlgaeWrist extends SubsystemBase {

  private final SparkMax motor = new SparkMax(AlgaeWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(
          AlgaeWristConstants.kP,
          AlgaeWristConstants.kI,
          AlgaeWristConstants.kD,
          AlgaeWristConstants.kConstraints);
  private final ArmFeedforward feedforward =
      new ArmFeedforward(AlgaeWristConstants.kS, AlgaeWristConstants.kG, AlgaeWristConstants.kV);

  private AlgaeWristState targetState = AlgaeWristState.Unknown;

  public AlgaeWrist() {
    // spotless:off
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEOCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);
    
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    
    config.absoluteEncoder
        .inverted(true) // TODO: determine if has any effect
        .zeroCentered(true)
        .zeroOffset(AlgaeWristConstants.kZeroOffset.in(Rotations))
        .positionConversionFactor(AlgaeWristConstants.kPositionConversionFactor.in(Radians))
        .velocityConversionFactor(AlgaeWristConstants.kVelocityConversionFactor.in(RadiansPerSecond));
    
    config.softLimit
        .reverseSoftLimit(AlgaeWristState.Min.angle.in(Radians))
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeWristState.Max.angle.in(Radians))
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    feedback.setTolerance(AlgaeWristConstants.kAllowableError.in(Radians));
    feedback.setIZone(AlgaeWristConstants.kIZone.in(Radians));
    feedback.enableContinuousInput(0, 2.0 * Math.PI); // TODO: determine if has any effect
    // feedback.setIntegratorRange();

    // setDefaultCommand(createRemainAtCurrentAngleCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Wrist/Current Angle Degrees", getCurrentAngleDegrees());
    SmartDashboard.putNumber("Algae Wrist/Current Angle Radians", encoder.getPosition());
    SmartDashboard.putNumber("Algae Wrist/Current Angular Velocity RPS", encoder.getVelocity());
    SmartDashboard.putString("Algae Wrist/Target State", getTargetState().name());
    SmartDashboard.putNumber("Algae Wrist/Setpoint Angle Degrees", getSetpointAngleDegrees());
    SmartDashboard.putNumber("Algae Wrist/Setpoint Angle Radians", feedback.getSetpoint().position);
    SmartDashboard.putNumber(
        "Algae Wrist/Setpoint Angular Velocity RPS", feedback.getSetpoint().velocity);
    SmartDashboard.putNumber("Algae Wrist/Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Algae Wrist/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Algae Wrist/At Goal", feedback.atGoal());
  }

  private double getCurrentAngleDegrees() {
    return Radians.of(encoder.getPosition()).in(Degrees);
  }

  private double getSetpointAngleDegrees() {
    return Radians.of(feedback.getSetpoint().position).in(Degrees);
  }

  public void resetController() {
    feedback.reset(encoder.getPosition(), encoder.getVelocity());
  }

  private void control() {
    double currentPosition = encoder.getPosition();
    double offsetPosition =
        currentPosition + AlgaeWristConstants.kCenterOfGravityOffset.in(Radians);
    double currentVelocitySetpoint = feedback.getSetpoint().velocity;
    motor.setVoltage(
        feedforward.calculate(offsetPosition, currentVelocitySetpoint)
            + feedback.calculate(currentPosition));
  }

  public AlgaeWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(AlgaeWristState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          feedback.setGoal(targetState.angle.in(Radians));
        },
        // execute
        () -> control(),
        // end
        interrupted -> {},
        // isFinished
        () -> feedback.atGoal(),
        // requirements
        this);
  }

  public Command createRemainAtCurrentAngleCommand() {
    return new FunctionalCommand(
        // initialize
        () -> {
          if (targetState == AlgaeWristState.Unknown) feedback.setGoal(encoder.getPosition());
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

  public Command createJoystickControlCommand(XboxController gamepad) {
    return this.run(
        () -> {
          double joystickInput = MathUtil.applyDeadband(-gamepad.getLeftY(), 0.05);
          motor.setVoltage(joystickInput);
        });
  }
}
