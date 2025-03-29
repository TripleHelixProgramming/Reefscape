package frc.robot.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  public void resetPositionControllers() {
    lifter.resetController();
    coralWrist.resetController();
    algaeWrist.resetController();
  }

  public Command coralL4PositionCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.CoralL4).schedule();
          coralWrist.setAngle(CoralWristState.L4).schedule();
          algaeWrist.setAngle(AlgaeWristState.CoralMode).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.getCurrentCommand().cancel();
        });
  }

  public Command coralL3PositionCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.CoralL3).schedule();
          coralWrist.setAngle(CoralWristState.L3).schedule();
          algaeWrist.setAngle(AlgaeWristState.CoralMode).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.getCurrentCommand().cancel();
        });
  }

  public Command coralL2PositionCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.CoralL2).schedule();
          coralWrist.setAngle(CoralWristState.L2).schedule();
          algaeWrist.setAngle(AlgaeWristState.CoralMode).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.getCurrentCommand().cancel();
        });
  }

  public Command coralL1PositionCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.CoralL1).schedule();
          coralWrist.setAngle(CoralWristState.L1).schedule();
          algaeWrist.setAngle(AlgaeWristState.CoralMode).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.getCurrentCommand().cancel();
        });
  }

  public Command coralIntakeCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.CoralIntake).schedule();
          coralWrist.setAngle(CoralWristState.Intake).schedule();
          algaeWrist.setAngle(AlgaeWristState.CoralMode).schedule();
          coralRoller.createIntakeCommand().schedule();
          algaeRoller.getCurrentCommand().cancel();
        });
  }

  public Command algaeBargePositionCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.AlgaeBarge).schedule();
          coralWrist.setAngle(CoralWristState.AlgaeMode).schedule();
          algaeWrist.setAngle(AlgaeWristState.Barge).schedule();
          coralRoller.getCurrentCommand().cancel();
        });
  }

  public Command algaeL3IntakeCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.AlgaeL3).schedule();
          coralWrist.setAngle(CoralWristState.AlgaeMode).schedule();
          algaeWrist.setAngle(AlgaeWristState.L3).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.createIntakeCommand().schedule();
        });
  }

  public Command algaeL2IntakeCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.AlgaeL2).schedule();
          coralWrist.setAngle(CoralWristState.AlgaeMode).schedule();
          algaeWrist.setAngle(AlgaeWristState.L2).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.createIntakeCommand().schedule();
        });
  }

  public Command algaeProcessorPositionCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.AlgaeProcessor).schedule();
          coralWrist.setAngle(CoralWristState.AlgaeMode).schedule();
          algaeWrist.setAngle(AlgaeWristState.Processor).schedule();
          coralRoller.getCurrentCommand().cancel();
        });
  }

  public Command algaeFloorIntakeCG() {
    return new InstantCommand(
        () -> {
          lifter.setHeight(LifterState.AlgaeIntakeFloor).schedule();
          coralWrist.setAngle(CoralWristState.AlgaeMode).schedule();
          algaeWrist.setAngle(AlgaeWristState.Floor).schedule();
          coralRoller.getCurrentCommand().cancel();
          algaeRoller.createIntakeCommand().schedule();
        });
  }

  public Command holdAlgaeCG() {
    return new InstantCommand(
        () -> {
          algaeRoller.holdAlgae().schedule();
          coralWrist.setAngle(CoralWristState.AlgaeMode).schedule();
          algaeWrist.setAngle(AlgaeWristState.Barge).schedule();
        });
  }
}
