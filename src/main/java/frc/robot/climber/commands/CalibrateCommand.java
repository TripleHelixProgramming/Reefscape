// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.CalibrationState;
import frc.robot.climber.Actuator;
import frc.robot.climber.Climber;

public class CalibrateCommand extends Command {

  private final Climber m_climber;
  private final Actuator[] m_actuators;

  public CalibrateCommand(Climber subsystem) {
    m_climber = subsystem;
    addRequirements(m_climber);
    m_actuators = m_climber.getClimberSides();
  }

  @Override
  public void initialize() {
    for (Actuator actuator : m_actuators) {
      actuator.configureUpperLimit(false);
      actuator.configurePositionController(
          ClimberConstants.slowConstraints, ClimberConstants.kSeekPosition);
      actuator.setCalibrationState(CalibrationState.UNCALIBRATED);
    }
  }

  @Override
  public void execute() {
    for (Actuator actuator : m_actuators) {
      if (actuator.getCalibrationState() != CalibrationState.CALIBRATED) calibrate(actuator);
    }
  }

  private void calibrate(Actuator actuator) {
    if (actuator.getCurrentSenseState()) {
      actuator.stop();
      actuator.resetEncoder();
      actuator.setCalibrationState(CalibrationState.CALIBRATED);
    } else {
      actuator.driveToTargetPosition();
      actuator.setCalibrationState(CalibrationState.HOMING);
    }
  }

  @Override
  public boolean isFinished() {
    for (Actuator actuator : m_actuators)
      if (actuator.getCalibrationState() != CalibrationState.CALIBRATED) return false;
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    for (Actuator actuator : m_actuators) actuator.configureUpperLimit(true);
  }
}
