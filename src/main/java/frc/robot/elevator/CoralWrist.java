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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.CoralWristConstants;
import frc.robot.elevator.ElevatorConstants.CoralWristConstants.CoralWristState;

public class CoralWrist extends SubsystemBase {

  private final SparkMax motor = new SparkMax(CoralWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          CoralWristConstants.kP,
          CoralWristConstants.kI,
          CoralWristConstants.kD,
          CoralWristConstants.kConstraints);

  private CoralWristState targetState = CoralWristState.Unknown;

  public CoralWrist() {
    // spotless:off
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    
    config.absoluteEncoder
        .inverted(false)
        .zeroCentered(false)
        .zeroOffset(CoralWristConstants.kPositionOffset.in(Rotations))
        .positionConversionFactor(CoralWristConstants.kPositionConversionFactor.in(Radians))
        .velocityConversionFactor(CoralWristConstants.kVelocityConversionFactor.in(RadiansPerSecond));

    config.softLimit
        .reverseSoftLimit(CoralWristState.Min.angle.in(Radians))
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(CoralWristState.Max.angle.in(Radians))
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller.setTolerance(CoralWristConstants.kAllowableAngleError.in(Radians));
    controller.setIZone(CoralWristConstants.kIZone.in(Radians));
    controller.enableContinuousInput(0, 2.0 * Math.PI);
    // controller.setIntegratorRange();

    setDefaultCommand(createRemainAtCurrentAngleCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Wrist/Current Angle Degrees", getCurrentAngleDegrees());
    SmartDashboard.putNumber("Coral Wrist/Current Angle Radians", encoder.getPosition());
    SmartDashboard.putNumber("Coral Wrist/Current Angular Velocity RPS", encoder.getVelocity());
    SmartDashboard.putString("Coral Wrist/Target State", getTargetState().name());
    SmartDashboard.putNumber("Coral Wrist/Setpoint Angle Degrees", getSetpointAngleDegrees());
    SmartDashboard.putNumber(
        "Coral Wrist/Setpoint Angle Radians", controller.getSetpoint().position);
    SmartDashboard.putNumber(
        "Coral Wrist/Setpoint Angular Velocity RPS", controller.getSetpoint().velocity);
    SmartDashboard.putNumber("Coral Wrist/Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Coral Wrist/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Coral Wrist/At Goal", controller.atGoal());
  }

  private double getCurrentAngleDegrees() {
    return Radians.of(encoder.getPosition()).in(Degrees);
  }

  private double getSetpointAngleDegrees() {
    return Radians.of(controller.getSetpoint().position).in(Degrees);
  }

  public void resetController() {
    controller.reset(encoder.getPosition(), encoder.getVelocity());
  }

  public void control() {
    motor.setVoltage(
        controller.calculate(encoder.getPosition())
            + CoralWristConstants.kG * Math.cos(encoder.getPosition()));
  }

  public CoralWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(CoralWristState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          controller.setGoal(targetState.angle.in(Radians));
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

  public Command createRemainAtCurrentAngleCommand() {
    return new FunctionalCommand(
        // initialize
        () -> {
          if (targetState == CoralWristState.Unknown) controller.setGoal(encoder.getPosition());
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
}
