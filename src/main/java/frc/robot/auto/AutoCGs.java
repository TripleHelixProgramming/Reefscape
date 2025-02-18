package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeIntakeConstants.AlgaeIntakeStates;
import frc.robot.Constants.CoralIntakeConstants.CoralIntakeStates;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeIntake;
import frc.robot.grippers.CoralIntake;

public class AutoCGs {

  Elevator elevator;
  CoralIntake coralIntake;
  AlgaeIntake algaeIntake;

  public AutoCGs(
      Elevator elevatorsubsystem,
      CoralIntake coralIntakeSubsystem,
      AlgaeIntake algaeIntakeSubsystem) {
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
  }

  public ParallelCommandGroup coralL4PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L4),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L4.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralL3PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L3),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L3.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralL2PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L2),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L2.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralL1PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.L1),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L1.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));
  }

  public ParallelCommandGroup coralIntakePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Intake),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.Intake.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));
  }

  public ParallelCommandGroup algaeBargePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Max),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Barge.angle));
  }

  public ParallelCommandGroup algaeL3PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.AlgaeL3),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.L3.angle));
  }

  public ParallelCommandGroup algaeL2PositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.AlgaeL2),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.L2.angle));
  }

  public ParallelCommandGroup algaeProcessorPositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Processor),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Processor.angle));
  }

  public ParallelCommandGroup algaeFloorIntakePositionCommand() {
    return new ParallelCommandGroup(
        elevator.createSetPositionCommand(ElevatorState.Floor),
        coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
        algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Floor.angle));
  }
}
