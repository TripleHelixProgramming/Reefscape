package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public CoralWrist getCoralWrist() {
    return coralWrist;
  }

  public AlgaeWrist getAlgaeWrist() {
    return algaeWrist;
  }

  public Command resetPositionControllers() {
    return new InstantCommand(
            () -> {
              lifter.resetController();
              coralWrist.resetController();
              algaeWrist.resetController();
            })
        .withName("Reset the position controllers");
  }

  public Command coralL4PositionCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.CoralL4),
            coralWrist.createSetAngleCommand(CoralWristState.L4),
            algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode))
        .withName("Elevator set to coralL4Position");
  }

  public Command coralL3PositionCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.CoralL3),
            coralWrist.createSetAngleCommand(CoralWristState.L3),
            algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode))
        .withName("Elevator set to coralL3Position");
  }

  public Command coralL2PositionCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.CoralL2),
            coralWrist.createSetAngleCommand(CoralWristState.L2),
            algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode))
        .withName("Elevator set to coralL2Position");
  }

  public Command coralL1PositionCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.CoralL1),
            coralWrist.createSetAngleCommand(CoralWristState.L1),
            algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode))
        .withName("Elevator set to coralL1Position");
  }

  public Command coralIntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.CoralIntake),
            coralWrist.createSetAngleCommand(CoralWristState.Intake),
            algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode))
        .withName("Elevator set to coral Intake position");
  }

  public Command algaeBargePositionCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.Max),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.Barge))
        .withName("Elevator set to algae Barge Position");
  }

  public Command algaeL3IntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeL3),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.L3),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge))
        .withName("Elevator set to algaeL3Intake position");
  }

  public Command algaeL2IntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeL2),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.L2),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge))
        .withName("Elevator set to algaeL2Intake position");
  }

  public Command algaeProcessorPositionCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeProcessor),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.Processor))
        .withName("Elevator set to algae processor position");
  }

  public Command algaeFloorIntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeIntakeFloor),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.Floor),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge))
        .withName("Elevator set to algae floor intake position");
  }
}
