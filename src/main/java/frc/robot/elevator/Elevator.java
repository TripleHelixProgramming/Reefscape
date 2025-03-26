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
          lifter.resetController();
          coralWrist.resetController();
          algaeWrist.resetController();
        });
  }

  public Command coralL4PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL4),
        coralWrist.setAngle(CoralWristState.L4),
        algaeWrist.setAngle(AlgaeWristState.CoralMode));
  }

  public Command coralL3PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL3),
        coralWrist.setAngle(CoralWristState.L3),
        algaeWrist.setAngle(AlgaeWristState.CoralMode));
  }

  public Command coralL2PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL2),
        coralWrist.setAngle(CoralWristState.L2),
        algaeWrist.setAngle(AlgaeWristState.CoralMode));
  }

  public Command coralL1PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL1),
        coralWrist.setAngle(CoralWristState.L1),
        algaeWrist.setAngle(AlgaeWristState.CoralMode));
  }

  public Command coralIntakeCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralIntake),
        coralWrist.setAngle(CoralWristState.Intake),
        algaeWrist.setAngle(AlgaeWristState.CoralMode));
  }

  public Command algaeBargePositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.AlgaeBarge),
        coralWrist.setAngle(CoralWristState.AlgaeMode),
        algaeWrist.setAngle(AlgaeWristState.Barge));
  }

  public Command algaeL3IntakeCG() {
    return Commands.parallel(
            lifter.setHeight(LifterState.AlgaeL3),
            coralWrist.setAngle(CoralWristState.AlgaeMode),
            algaeWrist.setAngle(AlgaeWristState.L3),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.setAngle(AlgaeWristState.Barge));
  }

  public Command algaeL2IntakeCG() {
    return Commands.parallel(
            lifter.setHeight(LifterState.AlgaeL2),
            coralWrist.setAngle(CoralWristState.AlgaeMode),
            algaeWrist.setAngle(AlgaeWristState.L2),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.setAngle(AlgaeWristState.Barge));
  }

  public Command algaeProcessorPositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.AlgaeProcessor),
        coralWrist.setAngle(CoralWristState.AlgaeMode),
        algaeWrist.setAngle(AlgaeWristState.Processor));
  }

  public Command algaeFloorIntakeCG() {
    return Commands.parallel(
            lifter.setHeight(LifterState.AlgaeIntakeFloor),
            coralWrist.setAngle(CoralWristState.AlgaeMode),
            algaeWrist.setAngle(AlgaeWristState.Floor),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae))
        .andThen(algaeWrist.setAngle(AlgaeWristState.Barge));
  }
}
