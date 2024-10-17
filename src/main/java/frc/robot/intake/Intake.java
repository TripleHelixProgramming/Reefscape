package frc.robot.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {

  private IntakeState m_state;

  private final CANSparkMax m_motor;

  private final RelativeEncoder m_relativeEncoder;

  private final SparkPIDController m_velocityController;

  private final ProfiledPIDController m_positionController =
      new ProfiledPIDController(
          IntakeConstants.kPositionP,
          IntakeConstants.kPositionI,
          IntakeConstants.kPositionD,
          IntakeConstants.kConstraints);

  private final DigitalInput m_noteSensorBeamBreak =
      new DigitalInput(IntakeConstants.kBeamBreakSensorDIOPort);
  private final DigitalInput m_noteSensorRetroReflective =
      new DigitalInput(IntakeConstants.kRetroReflectiveSensorDIOPort);

  private final EventLoop m_loop = new EventLoop();
  private final BooleanEvent m_secondSensorTriggered =
      new BooleanEvent(m_loop, secondSensorSupplier());

  public Intake() {

    m_state = IntakeState.IDLE;

    m_motor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_motor.setInverted(false);

    m_velocityController = m_motor.getPIDController();
    m_velocityController.setP(IntakeConstants.kVelocityP);
    m_velocityController.setI(IntakeConstants.kVelocityI);
    m_velocityController.setD(IntakeConstants.kVelocityD);

    m_positionController.setTolerance(IntakeConstants.kPositionTolerance);

    m_relativeEncoder = m_motor.getEncoder();
    m_relativeEncoder.setPosition(0.0);
    m_relativeEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);
    m_relativeEncoder.setVelocityConversionFactor(IntakeConstants.kVelocityConversionFactor);
  }

  public BooleanSupplier stateChecker(IntakeState state) {
    return () -> {
      if (this.m_state != null) return this.m_state.equals(state);
      else return false;
    };
  }

  private void setState(IntakeState state) {
    this.m_state = state;
  }

  public Command createStopIntakeCommand() {
    return this.runOnce(
        () -> {
          setState(IntakeState.IDLE);
          m_motor.set(0.0);
        });
  }

  private void configurePositionController(double targetPosition) {
    m_positionController.reset(m_relativeEncoder.getPosition(), m_relativeEncoder.getVelocity());
    m_positionController.setGoal(m_relativeEncoder.getPosition() + targetPosition);
  }

  // spotless:off
  private void advanceAfterIntaking(double targetPosition) {
    m_secondSensorTriggered.rising().ifHigh(
            () -> {
              m_positionController.reset(
                  m_relativeEncoder.getPosition(), m_relativeEncoder.getVelocity());
              m_positionController.setGoal(m_relativeEncoder.getPosition() + targetPosition);
            });
    setState(IntakeState.PROCESSING);
    m_motor.set(m_positionController.calculate(m_relativeEncoder.getPosition()));
  }
  // spotless:on

  public Command createIntakeCommand() {
    return this.run(
        () -> {
          setState(IntakeState.INTAKING);
          m_motor.set(1);
        });
  }

  public Command createOuttakeToAmpCommand() {
    return this.run(
        () -> {
          setState(IntakeState.OUTTAKING);
          m_motor.set(1.0);
        });
  }

  public Command createOuttakeToFloorCommand() {
    return this.run(
        () -> {
          setState(IntakeState.OUTTAKING);
          m_motor.set(-1.0);
        });
  }

  public Command createAdvanceAfterIntakingCommand() {
    return new FunctionalCommand(
        // initialize
        () -> this.configurePositionController(IntakeConstants.kFirstRepositionDistance),
        // execute
        () -> this.advanceAfterIntaking(IntakeConstants.kSecondRepositionDistance),
        // end
        interrupted -> {},
        // isFinished
        this.atGoalSupplier(),
        // requirements
        this);
  }

  public BooleanSupplier eitherSensorSupplier() {
    return () -> (!m_noteSensorRetroReflective.get() || !m_noteSensorBeamBreak.get());
  }

  public BooleanSupplier secondSensorSupplier() {
    return () -> !m_noteSensorRetroReflective.get();
  }

  public BooleanSupplier atGoalSupplier() {
    return () -> m_positionController.atGoal();
  }

  private void setVelocity(double targetVelocity) {
    m_velocityController.setReference(targetVelocity, ControlType.kVelocity);
  }

  public Command createSetVelocityCommand(double targetVelocity) {
    return this.startEnd(() -> this.setVelocity(targetVelocity), () -> {});
  }

  public Command createJoystickControlCommand(XboxController m_controller, double factor) {
    return this.run(
        () -> {
          setState(IntakeState.MANUALLY_REPOSITIONING);
          this.setVelocity(m_controller.getLeftY() * factor);
        });
  }

  @Override
  public void periodic() {
    m_loop.poll();

    SmartDashboard.putNumber("OutputCurrent", m_motor.getOutputCurrent());
    SmartDashboard.putNumber("IntakePosition", m_relativeEncoder.getPosition());
    SmartDashboard.putNumber("IntakeSetPoint", m_positionController.getSetpoint().position);
    SmartDashboard.putNumber("IntakeGoal", m_positionController.getGoal().position);
    SmartDashboard.putNumber(
        "IntakeSensorRetroReflective", !m_noteSensorRetroReflective.get() ? 1d : 0d);
    SmartDashboard.putNumber("IntakeSensorBeamBreak", !m_noteSensorBeamBreak.get() ? 1d : 0d);

    if (m_state != null) SmartDashboard.putString("Intake State", m_state.name());
  }
}
