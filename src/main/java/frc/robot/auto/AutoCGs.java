package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeWristConstants.AlgaeWristState;
import frc.robot.Constants.CoralWristConstants.CoralWristState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeRoller;
import frc.robot.grippers.AlgaeWrist;
import frc.robot.grippers.CoralRoller;
import frc.robot.grippers.CoralWrist;

public class AutoCGs {

  Elevator elevator;
  CoralWrist coralWrist;
  CoralRoller coralRoller;
  AlgaeWrist algaeWrist;
  AlgaeRoller algaeRoller;

  public AutoCGs(
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

  public ParallelCommandGroup coralL4PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L4),
        coralWrist.createSetAngleCommand(CoralWristState.L4),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.CoralMode.angle));
  }

  public ParallelCommandGroup coralL3PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L3),
        coralWrist.createSetAngleCommand(CoralWristState.L3),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.CoralMode.angle));
  }

  public ParallelCommandGroup coralL2PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L2),
        coralWrist.createSetAngleCommand(CoralWristState.L2),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.CoralMode.angle));
  }

  public ParallelCommandGroup coralL1PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L1),
        coralWrist.createSetAngleCommand(CoralWristState.L1),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.CoralMode.angle));
  }

  public ParallelCommandGroup coralIntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Intake),
        coralWrist.createSetAngleCommand(CoralWristState.Intake),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.CoralMode.angle),
        coralRoller.createSetIntakeCommand().until(coralRoller.hasCoral));
  }

  public ParallelCommandGroup algaeBargePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Max),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.Barge.angle));
  }

  public ParallelCommandGroup algaeL3IntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.AlgaeL3),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.L3.angle),
        algaeRoller.createSetIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeL2IntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.AlgaeL2),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.L2.angle),
        algaeRoller.createSetIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeProcessorPositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Processor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.Processor.angle));
  }

  public ParallelCommandGroup algaeFloorIntakeCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Floor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetRotationPositionCommand(AlgaeWristState.Floor.angle),
        algaeRoller.createSetIntakeCommand().until(algaeRoller.hasAlage));
  }
}
