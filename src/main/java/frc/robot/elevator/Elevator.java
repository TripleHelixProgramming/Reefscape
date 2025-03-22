package frc.robot.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  AlgaeRoller algaeRoller = new AlgaeRoller();
  AlgaeWrist algaeWrist = new AlgaeWrist(algaeRoller.hasAlgae);

  public Elevator() {}

  public void periodic() {
    SmartDashboard.putData(lifter);
    SmartDashboard.putData(coralWrist);
    SmartDashboard.putData(coralRoller);
    SmartDashboard.putData(algaeRoller);
    SmartDashboard.putData(algaeWrist);
  }

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
          lifter.matchHeight();
          lifter.resetController();
          coralWrist.resetController();
          algaeWrist.resetController();
        });
  }

  public Command coralL4PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.CoralL4),
        coralWrist.createSetAngleCommand(CoralWristState.L4),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralL3PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.CoralL3),
        coralWrist.createSetAngleCommand(CoralWristState.L3),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralL2PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.CoralL2),
        coralWrist.createSetAngleCommand(CoralWristState.L2),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralL1PositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.CoralL1),
        coralWrist.createSetAngleCommand(CoralWristState.L1),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command coralIntakeCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.CoralIntake),
        coralWrist.createSetAngleCommand(CoralWristState.Intake),
        algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode));
  }

  public Command algaeBargePositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.AlgaeBarge),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public Command algaeL3IntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeL3),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.L3),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public Command algaeL2IntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeL2),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.L2),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }

  public Command algaeProcessorPositionCG() {
    return Commands.parallel(
        lifter.createSetHeightCommand(LifterState.AlgaeProcessor),
        coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
        algaeWrist.createSetAngleCommand(AlgaeWristState.Processor));
  }

  public Command algaeFloorIntakeCG() {
    return Commands.parallel(
            lifter.createSetHeightCommand(LifterState.AlgaeIntakeFloor),
            coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode),
            algaeWrist.createSetAngleCommand(AlgaeWristState.Floor),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
  }
}
