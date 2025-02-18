package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeWristConstants.AlgaeWristState;
import frc.robot.Constants.CoralWristConstants.CoralWristState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

public class ElevatorCGs {

  Elevator elevator;
  CoralWrist coralWrist;
  CoralRoller coralRoller;
  AlgaeWrist algaeWrist;
  AlgaeRoller algaeRoller;

  public ElevatorCGs(
      Elevator elevatorsubsystem,
      CoralWrist coralWristSubsystem,
      CoralRoller coralRollerSubsystem,
      AlgaeWrist algaeWristSubsystem,
      AlgaeRoller algaeRollerSubsystem) {
    elevator = elevatorsubsystem;
    coralWrist = coralWristSubsystem;
    coralRoller = coralRollerSubsystem;
    algaeWrist = algaeWristSubsystem;
    algaeRoller = algaeRollerSubsystem;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public CoralRoller getCoralRoller() {
    return coralRoller;
  }

  public AlgaeRoller getAlgaeRoller() {
    return algaeRoller;
  }

  public ParallelCommandGroup coralL4PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.L4),
        coralWrist.createSetAngleCommand(CoralWristState.L4),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL3PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.L3),
        coralWrist.createSetAngleCommand(CoralWristState.L3),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL2PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.L2),
        coralWrist.createSetAngleCommand(CoralWristState.L2),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL1PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.L1),
        coralWrist.createSetAngleCommand(CoralWristState.L1),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralIntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.Intake),
        coralWrist.createSetAngleCommand(CoralWristState.Intake),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode),
        coralRoller.createSetIntakeCommand().until(coralRoller.hasCoral));
  }

  public ParallelCommandGroup algaeBargePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.Max),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public ParallelCommandGroup algaeL3IntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.AlgaeL3),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L3),
        algaeRoller.createSetIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeL2IntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.AlgaeL2),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L2),
        algaeRoller.createSetIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeProcessorPositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.Processor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Processor));
  }

  public ParallelCommandGroup algaeFloorIntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetHeightCommand(ElevatorState.Floor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Floor),
        algaeRoller.createSetIntakeCommand().until(algaeRoller.hasAlage));
  }
}
