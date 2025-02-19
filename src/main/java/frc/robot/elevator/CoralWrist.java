package frc.robot.elevator;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.event.EventLoop;
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
  // private final SparkClosedLoopController controller = motor.getClosedLoopController();

  // TODO: Add ArmFeedforward
  // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ArmFeedforward.html
  private final ProfiledPIDController controller =
      new ProfiledPIDController(CoralWristConstants.kP, CoralWristConstants.kI, 0.0,
  CoralWristConstants.kConstraints);

  private final EventLoop loop = new EventLoop();
  private CoralWristState targetState = CoralWristState.Unknown;
  private Angle targetAngle = targetState.angle;

  public CoralWrist() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    // config.closedLoop
    //     .p(CoralWristConstants.kP)
    //     .i(0.0)
    //     .d(0.0)
    //     // .izone(),
    //     // .outputRange(-1, 1)
    //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    
    config.absoluteEncoder
        .positionConversionFactor(CoralWristConstants.kPositionConversionFactor)
        .zeroOffset(CoralWristConstants.kPositionOffset)
        .inverted(false);

    config.softLimit
        .reverseSoftLimit(CoralWristState.Min.angle.in(Degrees))
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(CoralWristState.Max.angle.in(Degrees))
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller.setTolerance(CoralWristConstants.kAllowableAngleError.in(Degrees));
    // controller.setIZone();
    // controller.setIntegratorRange();

    // setDefaultCommand(createRemainAtCurrentAngleCommand());
    setDefaultCommand(this.startEnd(() -> motor.set(0.0), () -> {}));
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Coral Wrist Angle", encoder.getPosition());
    SmartDashboard.putString("Coral Wrist Target State", getTargetState().name());
    SmartDashboard.putNumber("Coral Wrist Target Angle", controller.getGoal().position);
    // SmartDashboard.putNumber("Coral Wrist Target Angle", targetAngle.in(Degrees));
    SmartDashboard.putNumber("Coral Wrist Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Coral Wrist Current", motor.getOutputCurrent());

    SmartDashboard.putBoolean("Coral Wrist At Goal", controller.atGoal());
  }

  public void resetController() {
    controller.reset(encoder.getPosition(), encoder.getVelocity());
  }

  public void control() {
    motor.set(controller.calculate(encoder.getPosition()));
  }

  public CoralWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(CoralWristState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          targetAngle = state.angle;
          controller.setGoal(targetState.angle.in(Degrees));
          // controller.setReference(targetState.angle.in(Degrees), ControlType.kPosition);
        },
        // execute
        () -> {}, // control()
        // end
        interrupted -> {
          System.err.println("Set angle command ended " + interrupted);
        },
        // isFinished
        () -> false, // controller.atGoal()
        // requirements
        this);
  }

  public Command createRemainAtCurrentAngleCommand() {
    return new FunctionalCommand(
        // initialize
        () -> {
          if (targetState == CoralWristState.Unknown) controller.setGoal(encoder.getPosition());
          // if (targetState == CoralWristState.Unknown)
          //   controller.setReference(targetState.angle.in(Degrees), ControlType.kPosition);
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
