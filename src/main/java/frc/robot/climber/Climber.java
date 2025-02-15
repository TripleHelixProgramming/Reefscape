package frc.robot.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ClimberConstants.ClimberController;

public class Climber extends SubsystemBase {
  private final SparkFlex motor =
      new SparkFlex(ClimberConstants.kClimberPort, MotorType.kBrushless);
  private final SparkFlexConfig motorConfig = new SparkFlexConfig();
  private final RelativeEncoder encoder = motor.getEncoder();

  private final ProfiledPIDController controller = new ProfiledPIDController(ClimberController.kP, ClimberController.kI, ClimberController.kD, ClimberController.kConstraints);

  private final Servo servo = new Servo(ClimberConstants.kRatchetServoPort);

  private final DigitalInput cageSensor = new DigitalInput(ClimberConstants.kCageSensorPort);

  public Climber() {
    motorConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.kClimberCurrentLimit);

    motorConfig
        .encoder
        .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
        .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

    motorConfig.softLimit.reverseSoftLimit(0.0).reverseSoftLimitEnabled(true);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller.setTolerance(ClimberConstants.kClimberTolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getPosition());
    SmartDashboard.putBoolean("Cage Sensor", cageSensor.get());

    SmartDashboard.putNumber("Servo", servo.get());

    SmartDashboard.putNumber("Climber Motor Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("isfinished for Command", createDeployCommand().isFinished());
    SmartDashboard.putNumber("Output Voltage Climber", motor.get());
  }

  public void reset() {
    encoder.setPosition(0);
    controller.reset(encoder.getPosition());
    controller.setGoal(encoder.getPosition());
    lockRatchet();
  }

  public void unlockRatchet() {
    servo.set(ClimberConstants.kDisengedPosition);
  }

  public void lockRatchet() {
    servo.set(ClimberConstants.kEngagedPosition);
  }

  private Boolean isInRange(double position) {
    if (position < 0.0) return false;
    return true;
  }

  public Command createClimbByJoystickCommand(XboxController gamepad) {
    return this.run(
      () -> {
        double targetPosition = controller.getGoal().position;
        double joystickInput = MathUtil.applyDeadband(-gamepad.getRightY(), 0.05);
        targetPosition += Math.max(0.0, joystickInput) * ClimberConstants.kMaxVelocity * RobotConstants.kPeriod;
        if (isInRange(targetPosition)) controller.setGoal(targetPosition);
        motor.set(controller.calculate(encoder.getPosition()));
      });
  }

  private double getPosition() {
    return encoder.getPosition();
  }

  public Command createStopCommand() {
    return this.run(() -> motor.set(0.0));
  }

  /**
   * @return Command that unlocks and deploys the climber arm
   */
  public Command createDeployCommand() {
    return new FunctionalCommand(
        // initialize
        () -> {
          unlockRatchet();
          controller.setGoal(ClimberConstants.kDeployPosition);
        },
        // execute
        () -> {
          motor.set(controller.calculate(encoder.getPosition()));
        },
        // end
        interrupted -> {
          lockRatchet();
          motor.set(0.0);
        },
        // isFinished
        () -> controller.atGoal(),
        // requirements
        this);
  }
}
