package frc.robot.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeWristConstants.AlgaeWristState;
import frc.robot.Constants.CoralWristConstants.CoralWristState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

public class Elevator {

  Lifter lifter = new Lifter();
  CoralWrist coralWrist = new CoralWrist();
  CoralRoller coralRoller = new CoralRoller();
  AlgaeWrist algaeWrist = new AlgaeWrist();
  AlgaeRoller algaeRoller = new AlgaeRoller();

  public Elevator() {}

  public Lifter getLifter() {
    return lifter;
  }

  public CoralRoller getCoralRoller() {
    return coralRoller;
  }

  public AlgaeRoller getAlgaeRoller() {
    return algaeRoller;
  }

  public void setDefaultCommands(XboxController operator) {
    lifter.setDefaultCommand(lifter.createJoystickControlCommand(operator));
    algaeRoller.setDefaultCommand(algaeRoller.createStopCommand());
    coralRoller.setDefaultCommand(coralRoller.createStopCommand());
  }

  public void resetPositionControllers() {
    lifter.resetPositionController();
    coralWrist.resetPositionController();
    algaeWrist.resetPositionController();
  }

  public ParallelCommandGroup coralL4PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.L4),
        coralWrist.createSetAngleCommand(CoralWristState.L4),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL3PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.L3),
        coralWrist.createSetAngleCommand(CoralWristState.L3),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL2PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.L2),
        coralWrist.createSetAngleCommand(CoralWristState.L2),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL1PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.L1),
        coralWrist.createSetAngleCommand(CoralWristState.L1),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralIntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.Intake),
        coralWrist.createSetAngleCommand(CoralWristState.Intake),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode),
        coralRoller.createIntakeCommand().until(coralRoller.hasCoral));
  }

  public ParallelCommandGroup algaeBargePositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.Max),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public ParallelCommandGroup algaeL3IntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.AlgaeL3),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L3),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeL2IntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.AlgaeL2),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L2),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeProcessorPositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.Processor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Processor));
  }

  public ParallelCommandGroup algaeFloorIntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(ElevatorState.Floor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Floor),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }
}
