package frc.robot.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorControllerGains;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;

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
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
  }

  private void setPosition(ElevatorPosition position) {
    controller.setReference(position.height, ControlType.kPosition);
  }

  public Command createSetPositionCommand(ElevatorPosition position) {
    return new InstantCommand(() -> this.setPosition(position));
  }

  public Command createStopIntakeCommand() {
    return this.runOnce(
        () -> {
          leaderMotor.set(0.0);
          followerMotor.set(0.0);
        });
  }

}
