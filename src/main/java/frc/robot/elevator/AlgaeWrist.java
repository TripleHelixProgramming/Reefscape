package frc.robot.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
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

  private ExponentialProfile profile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              AlgaeWristConstants.kMaxOutput, AlgaeWristConstants.kV, AlgaeWristConstants.kA));
  private ArmFeedforward feedforward =
      new ArmFeedforward(AlgaeWristConstants.kS, AlgaeWristConstants.kG, AlgaeWristConstants.kV);
  private final PIDController feedback =
      new PIDController(AlgaeWristConstants.kP, AlgaeWristConstants.kI, AlgaeWristConstants.kD);

  private ExponentialProfile.State currentSetpoint = new ExponentialProfile.State();
  private ExponentialProfile.State goal = new ExponentialProfile.State();
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
        .reverseSoftLimit(AlgaeWristState.Min.angle.in(Radians) - (2.0 * Math.PI))
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(AlgaeWristState.Max.angle.in(Radians))
        .forwardSoftLimitEnabled(false);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    feedback.setTolerance(AlgaeWristConstants.kAllowableAngleError.in(Radians));
    feedback.setIZone(AlgaeWristConstants.kIZone.in(Radians));
    feedback.enableContinuousInput(0, 2.0 * Math.PI);
    // controller.setIntegratorRange();

    setDefaultCommand(createRemainAtCurrentAngleCommand());
    // setDefaultCommand(this.startEnd(() -> motor.setVoltage(0.5), () -> {}));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Wrist/Current Angle Degrees", getCurrentAngleDegrees());
    SmartDashboard.putNumber("Algae Wrist/Current Angle Radians", encoder.getPosition());
    SmartDashboard.putNumber("Algae Wrist/Current Angular Velocity RPS", encoder.getVelocity());
    SmartDashboard.putString("Algae Wrist/Target State", getTargetState().name());
    SmartDashboard.putNumber("Algae Wrist/Setpoint Angle Degrees", getSetpointAngleDegrees());
    SmartDashboard.putNumber("Algae Wrist/Setpoint Angle Radians", currentSetpoint.position);
    SmartDashboard.putNumber("Algae Wrist/Setpoint Angular Velocity RPS", currentSetpoint.velocity);
    SmartDashboard.putNumber("Algae Wrist/Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Algae Wrist/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Algae Wrist/At Goal", atGoal());
  }

  private double getCurrentAngleDegrees() {
    return Radians.of(encoder.getPosition()).in(Degrees);
  }

  private double getSetpointAngleDegrees() {
    return Radians.of(feedback.getSetpoint()).in(Degrees);
  }

  public void resetController() {
    feedback.reset();
  }

  private void setGoalRadians(double goal) {
    this.goal.position = goal;
  }

  private boolean atGoal() {
    return feedback.atSetpoint() && goal.equals(currentSetpoint);
  }

  private void controller() {
    var nextSetpoint = profile.calculate(RobotConstants.kPeriod.in(Seconds), currentSetpoint, goal);
    motor.setVoltage(
        feedforward.calculateWithVelocities(
                encoder.getPosition(), encoder.getVelocity(), nextSetpoint.velocity)
            // feedforward.calculate(nextSetpoint.position, nextSetpoint.velocity)
            + feedback.calculate(encoder.getPosition(), nextSetpoint.position));
    currentSetpoint = nextSetpoint;
  }

  public AlgaeWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(AlgaeWristState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          setGoalRadians(targetState.angle.in(Radians));
        },
        // execute
        () -> controller(),
        // end
        interrupted -> {},
        // isFinished
        () -> atGoal(),
        // requirements
        this);
  }

  public Command createRemainAtCurrentAngleCommand() {
    return new FunctionalCommand(
        // initialize
        () -> {
          if (targetState == AlgaeWristState.Unknown) setGoalRadians(encoder.getPosition());
        },
        // execute
        () -> controller(),
        // end
        interrupted -> {},
        // isFinished
        () -> false,
        // requirements
        this);
  }
}
