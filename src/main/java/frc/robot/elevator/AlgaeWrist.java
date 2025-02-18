package frc.robot.elevator;

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
import frc.robot.Constants.AlgaeWristConstants;
import frc.robot.Constants.AlgaeWristConstants.AlgaeWristState;
import frc.robot.Constants.RobotConstants;

public class AlgaeWrist extends SubsystemBase {

  private final SparkMax motor = new SparkMax(AlgaeWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
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
        .reverseSoftLimit(AlgaeWristState.Min.angle)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeWristState.Max.angle)
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller.setTolerance(AlgaeWristConstants.kAllowableAngleError);
    // controller.setIZone();
    // controller.setIntegratorRange();
    resetPositionController();
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Algae Rotation Position", encoder.getPosition());
  }

  public void resetPositionController() {
    controller.reset(encoder.getPosition());
    controller.setGoal(encoder.getPosition());
  }

  public AlgaeWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(AlgaeWristState state) {
    return new FunctionalCommand(
        // initialize
        () -> {
          targetState = state;
          controller.setGoal(targetState.angle);
        },
        // execute
        () -> {
          motor.set(controller.calculate(encoder.getPosition()));
        },
        // end
        interrupted -> {},
        // isFinished
        () -> controller.atGoal(),
        // requirements
        this);
  }
}
