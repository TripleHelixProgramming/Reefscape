package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeWristConstants.AlgaeWristStates;
import frc.robot.Constants.CoralWristConstants.CoralWristStates;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeWrist;
import frc.robot.grippers.CoralRoller;
import frc.robot.grippers.CoralWrist;

public class AutoCGs {

  Elevator elevator;
  CoralWrist coralWrist;
  CoralRoller coralRoller;
  AlgaeWrist algaeWrist;

  public AutoCGs(
      Elevator elevatorsubsystem,
      CoralWrist coralWristSubsystem,
      CoralRoller coralRollerSubsystem,
      AlgaeWrist algaeWristSubsystem) {
    elevator = elevatorsubsystem;
    coralWrist = coralWristSubsystem;
    coralRoller = coralRollerSubsystem;
    algaeWrist = algaeWristSubsystem;
  }

  public ParallelCommandGroup coralL4PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L4),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.L4.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralL3PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L3),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.L3.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralL2PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L2),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.L2.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralL1PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L1),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.L1.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralIntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Intake),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.Intake.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.CoralMode.angle),
        coralRoller.createSetIntakeCommand().until(coralRoller.hasCoralPiece()));
  }

  public ParallelCommandGroup algaeBargePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Max),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.AlgaeMode.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.Barge.angle));
  }

  public ParallelCommandGroup algaeL3PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.AlgaeL3),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.AlgaeMode.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.L3.angle));
  }

  public ParallelCommandGroup algaeL2PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.AlgaeL2),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.AlgaeMode.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.L2.angle));
  }

  public ParallelCommandGroup algaeProcessorPositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Processor),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.AlgaeMode.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.Processor.angle));
  }

  public ParallelCommandGroup algaeFloorIntakePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Floor),
        coralWrist.createSetRotationPositionCommand(CoralWristStates.AlgaeMode.angle),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristStates.Floor.angle));
  }
}
