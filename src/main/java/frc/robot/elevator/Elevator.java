package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeWristConstants.AlgaeWristState;
import frc.robot.Constants.CoralWristConstants.CoralWristState;
import frc.robot.Constants.LifterConstants.LifterState;

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

  public void setRollerDefaultCommands() {
    algaeRoller.setDefaultCommand(algaeRoller.createStopCommand());
    coralRoller.setDefaultCommand(coralRoller.createStopCommand());
  }

  public void setWristDefaultCommands() {
    algaeWrist.setDefaultCommand(algaeWrist.createRemainAtCurrentAngleCommand());
    coralWrist.setDefaultCommand(coralWrist.createRemainAtCurrentAngleCommand());
  }

  public void resetPositionControllers() {
    lifter.resetController();
    coralWrist.resetController();
    algaeWrist.resetController();
  }

  public ParallelCommandGroup coralL4PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.L4),
        coralWrist.createSetAngleCommand(CoralWristState.L4),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL3PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.L3),
        coralWrist.createSetAngleCommand(CoralWristState.L3),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL2PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.L2),
        coralWrist.createSetAngleCommand(CoralWristState.L2),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralL1PositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.L1),
        coralWrist.createSetAngleCommand(CoralWristState.L1),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public ParallelCommandGroup coralIntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.Intake),
        coralWrist.createSetAngleCommand(CoralWristState.Intake),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode),
        coralRoller.createIntakeCommand().until(coralRoller.hasCoral));
  }

  public ParallelCommandGroup algaeBargePositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.Max),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public ParallelCommandGroup algaeL3IntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.AlgaeL3),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L3),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeL2IntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.AlgaeL2),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.L2),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }

  public ParallelCommandGroup algaeProcessorPositionCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.Processor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Processor));
  }

  public ParallelCommandGroup algaeFloorIntakeCommand() {
    return new ParallelCommandGroup(
        lifter.createSetHeightCommand(LifterState.Floor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Floor),
        algaeRoller.createIntakeCommand().until(algaeRoller.hasAlage));
  }
}
