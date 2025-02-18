package frc.robot.grippers;

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
import frc.robot.Constants.CoralWristConstants;
import frc.robot.Constants.CoralWristConstants.CoralWristState;
import frc.robot.Constants.RobotConstants;

public class CoralWrist extends SubsystemBase {

  private final SparkMax motor = new SparkMax(CoralWristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  private final ProfiledPIDController controller =
      new ProfiledPIDController(CoralWristConstants.kP, 0.0, 0.0, CoralWristConstants.kConstraints);

  private final EventLoop loop = new EventLoop();

  private CoralWristState targetState = CoralWristState.Unknown;

  public CoralWrist() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);
    
    config.absoluteEncoder
        .positionConversionFactor(CoralWristConstants.kPositionConversionFactor)
        .zeroOffset(CoralWristConstants.kPositionOffset)
        .inverted(false);

    config.softLimit
        .reverseSoftLimit(CoralWristState.Min.angle)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(CoralWristState.Max.angle)
        .forwardSoftLimitEnabled(true);
    // spotless:on

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller.setTolerance(CoralWristConstants.kAllowableAngleError);
    // controller.setIZone();
    // controller.setIntegratorRange();
    resetPositionController();
  }

  @Override
  public void periodic() {
    loop.poll();

    SmartDashboard.putNumber("Coral Rotation Position", encoder.getPosition());
  }

  public void resetPositionController() {
    controller.reset(encoder.getPosition());
    controller.setGoal(encoder.getPosition());
  }

  public CoralWristState getTargetState() {
    return this.targetState;
  }

  public Command createSetAngleCommand(CoralWristState state) {
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
