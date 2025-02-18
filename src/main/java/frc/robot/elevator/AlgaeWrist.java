package frc.robot.elevator;

import static edu.wpi.first.units.Units.Degrees;

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

  // TODO: Add ArmFeedforward
  // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ArmFeedforward.html
  private final ProfiledPIDController controller =
      new ProfiledPIDController(AlgaeWristConstants.kP, 0.0, 0.0, AlgaeWristConstants.kConstraints);

  private final EventLoop loop = new EventLoop();
  private AlgaeWristState targetState = AlgaeWristState.Unknown;

  public AlgaeWrist() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEOCurrentLimit)
        .inverted(false);
    
    config.absoluteEncoder
        .positionConversionFactor(AlgaeWristConstants.kPositionConversionFactor)
        .zeroOffset(AlgaeWristConstants.kPositionOffset)
        .inverted(true);
    
    config.softLimit
        .reverseSoftLimit(AlgaeWristState.Min.angle.in(Degrees))
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeWristState.Max.angle.in(Degrees))
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller.setTolerance(AlgaeWristConstants.kAllowableAngleError.in(Degrees));
    // controller.setIZone();
    // controller.setIntegratorRange();

    // setDefaultCommand(createRemainAtCurrentAngleCommand());
    setDefaultCommand(this.startEnd(() -> motor.set(0.0), () -> {}));
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Algae Wrist Angle", encoder.getPosition());
    SmartDashboard.putString("Algae Wrist Target State", getTargetState().name());
    SmartDashboard.putNumber("Algae Wrist Target Angle", controller.getGoal().position);
    SmartDashboard.putNumber("Algae Wrist Applied Duty Cycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Algae Wrist Current", motor.getOutputCurrent());
  }

  public void resetController() {
    controller.reset(encoder.getPosition(), encoder.getVelocity());
  }

  public void control() {
    motor.set(controller.calculate(encoder.getPosition()));
  }

  public AlgaeWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(AlgaeWristState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          controller.setGoal(targetState.angle.in(Degrees));
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
