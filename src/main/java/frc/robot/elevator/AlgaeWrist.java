package frc.robot.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.event.EventLoop;
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

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          AlgaeWristConstants.kP,
          AlgaeWristConstants.kI,
          AlgaeWristConstants.kD,
          AlgaeWristConstants.kConstraints);

  private final EventLoop loop = new EventLoop();
  private AlgaeWristState targetState = AlgaeWristState.Unknown;

  public AlgaeWrist() {
    // spotless:off
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEOCurrentLimit)
        .voltageCompensation(RobotConstants.kNominalVoltage);
        
    config.absoluteEncoder
        .inverted(true)
        .zeroCentered(true)
        .zeroOffset(AlgaeWristConstants.kPositionOffset)
        .positionConversionFactor(AlgaeWristConstants.kPositionConversionFactor)
        .velocityConversionFactor(AlgaeWristConstants.kVelocityConversionFactor);
    
    config.softLimit
        .reverseSoftLimit((AlgaeWristState.Min.angle.in(Radians)-AlgaeWristConstants.kPositionOffset)*Math.PI/2.0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit((AlgaeWristState.Max.angle.in(Radians)-AlgaeWristConstants.kPositionOffset)*Math.PI/2.0)
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller.setTolerance(AlgaeWristConstants.kAllowableAngleError.in(Radians));
    controller.setIZone(AlgaeWristConstants.kIZone.in(Radians));
    controller.enableContinuousInput(0, 2.0 * Math.PI);
    // controller.setIntegratorRange();

    setDefaultCommand(createRemainAtCurrentAngleCommand());
    // setDefaultCommand(this.startEnd(() -> motor.setVoltage(0.3), () -> {}));
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Algae Wrist/Current Angle Degrees", getCurrentAngleDegrees());
    SmartDashboard.putNumber("Algae Wrist/Current Angle Radians", encoder.getPosition());
    SmartDashboard.putNumber("Algae Wrist/Current Angular Velocity RPS", encoder.getVelocity());
    SmartDashboard.putString("Algae Wrist/Target State", getTargetState().name());
    SmartDashboard.putNumber("Algae Wrist/Setpoint Angle Degrees", getSetpointAngleDegrees());
    SmartDashboard.putNumber(
        "Algae Wrist/Setpoint Angle Radians", controller.getSetpoint().position);
    SmartDashboard.putNumber(
        "Algae Wrist/Setpoint Angular Velocity RPS", controller.getSetpoint().velocity);
    SmartDashboard.putNumber("Algae Wrist/Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Algae Wrist/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Algae Wrist/At Goal", controller.atGoal());
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

  private void control() {
    motor.setVoltage(
        controller.calculate(encoder.getPosition())
            + AlgaeWristConstants.kG * Math.cos(encoder.getPosition()));
  }

  public AlgaeWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(AlgaeWristState state) {
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
          if (targetState == AlgaeWristState.Unknown) controller.setGoal(encoder.getPosition());
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
