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
        lifter.setHeight(LifterState.CoralL4).asProxy(),
        coralWrist.setAngle(CoralWristState.L4).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.CoralMode).asProxy());
  }

  public Command coralL3PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL3).asProxy(),
        coralWrist.setAngle(CoralWristState.L3).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.CoralMode).asProxy());
  }

  public Command coralL2PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL2).asProxy(),
        coralWrist.setAngle(CoralWristState.L2).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.CoralMode).asProxy());
  }

  public Command coralL1PositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralL1).asProxy(),
        coralWrist.setAngle(CoralWristState.L1).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.CoralMode).asProxy());
  }

  public Command coralIntakeCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.CoralIntake).asProxy(),
        coralWrist.setAngle(CoralWristState.Intake).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.CoralMode).asProxy());
  }

  public Command algaeBargePositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.AlgaeBarge).asProxy(),
        coralWrist.setAngle(CoralWristState.AlgaeMode).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.Barge).asProxy());
  }

  public Command algaeL3IntakeCG() {
    return Commands.parallel(
            lifter.setHeight(LifterState.AlgaeL3).asProxy(),
            coralWrist.setAngle(CoralWristState.AlgaeMode).asProxy(),
            algaeWrist.setAngle(AlgaeWristState.L3).asProxy(),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae).asProxy())
        .andThen(algaeWrist.setAngle(AlgaeWristState.Barge).asProxy());
  }

  public Command algaeL2IntakeCG() {
    return Commands.parallel(
            lifter.setHeight(LifterState.AlgaeL2).asProxy(),
            coralWrist.setAngle(CoralWristState.AlgaeMode).asProxy(),
            algaeWrist.setAngle(AlgaeWristState.L2).asProxy(),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae).asProxy())
        .andThen(algaeWrist.setAngle(AlgaeWristState.Barge).asProxy());
  }

  public Command algaeProcessorPositionCG() {
    return Commands.parallel(
        lifter.setHeight(LifterState.AlgaeProcessor).asProxy(),
        coralWrist.setAngle(CoralWristState.AlgaeMode).asProxy(),
        algaeWrist.setAngle(AlgaeWristState.Processor).asProxy());
  }

  public Command algaeFloorIntakeCG() {
    return Commands.parallel(
            lifter.setHeight(LifterState.AlgaeIntakeFloor).asProxy(),
            coralWrist.setAngle(CoralWristState.AlgaeMode).asProxy(),
            algaeWrist.setAngle(AlgaeWristState.Floor).asProxy(),
            algaeRoller.createIntakeCommand().until(algaeRoller.hasAlgae).asProxy())
        .andThen(algaeWrist.setAngle(AlgaeWristState.Barge).asProxy());
  }
}
