package frc.robot.elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerGains;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;

public class Elevator extends SubsystemBase {

  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  /**
   * private final ProfiledPIDController m_positionController = new ProfiledPIDController(
   * IntakeConstants.kPositionP, IntakeConstants.kPositionI, IntakeConstants.kPositionD,
   * IntakeConstants.kConstraints);
   */
  private Elevator() {
    leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorPort, MotorType.kBrushless);
    followerMotor = new SparkMax(ElevatorConstants.kFollowerMotorPort, MotorType.kBrushless);

    motorConfig
      .voltageCompensation(RobotConstants.kNominalVoltage)
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);

    motorConfig.closedLoop.maxMotion
      .maxAcceleration(ElevatorConstants.kMaxAcceleration)
      .maxVelocity(ElevatorConstants.kMaxVelocity);

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(ElevatorControllerGains.kP)
      .i(ElevatorControllerGains.kI)
      .d(ElevatorControllerGains.kD)
      // .iZone()
      // .velocityFF(ElevatorConstants.kFF)
      .outputRange(-1, 1);

    motorConfig.encoder
      .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

    leaderMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    controller = leaderMotor.getClosedLoopController();
    encoder = leaderMotor.getEncoder();

    controller.setReference(encoder.getPosition(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // public information about the subsystem to SmartDashboard
  }

  public Command createStopIntakeCommand() {
    return this.runOnce(
        () -> {
          leaderMotor.set(0.0);
          followerMotor.set(0.0);
        });
  }

  public Command createSetElevationCommand(ElevatorHeight height) {
    return new FunctionalCommand(
        // initialize
        () -> {},
        // execute
        () -> {},
        // end
        interrupted -> {},
        // isFinished
        this.atGoalSupplier(),
        // requirements
        this);
  }

}
