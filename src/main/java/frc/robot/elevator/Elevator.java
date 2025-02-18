package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.elevator.ElevatorConstants.AlgaeWristConstants.AlgaeWristState;
import frc.robot.elevator.ElevatorConstants.CoralWristConstants.CoralWristState;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterState;

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

  public void resetPositionControllers() {
    lifter.resetController();
    coralWrist.resetController();
    algaeWrist.resetController();
  }

  public Command coralL4PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.L4),
        coralWrist.createSetAngleCommand(CoralWristState.L4),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralL3PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.L3),
        coralWrist.createSetAngleCommand(CoralWristState.L3),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralL2PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.L2),
        coralWrist.createSetAngleCommand(CoralWristState.L2),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralL1PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.L1),
        coralWrist.createSetAngleCommand(CoralWristState.L1),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralIntakeCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.Intake),
        coralWrist.createSetAngleCommand(CoralWristState.Intake),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode),
        coralRoller.createIntakeCommand().until(coralRoller.hasCoral));
  }

  public Command algaeBargePositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.Max),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public Command algaeL3IntakeCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.AlgaeL3),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L3),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }

  public Command algaeL2IntakeCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.AlgaeL2),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L2),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }

  public Command algaeProcessorPositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.Processor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Processor));
  }

  public Command algaeFloorIntakeCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.Floor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Floor),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }
}
