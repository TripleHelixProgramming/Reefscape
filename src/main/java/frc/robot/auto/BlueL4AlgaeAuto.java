package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorCGs;
import java.util.Optional;

public class BlueL4AlgaeAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  ElevatorCGs elevatorCG;

  public BlueL4AlgaeAuto(Drivetrain drivetrain, ElevatorCGs autoCommandGroups) {
    super(drivetrain);
    elevatorCG = autoCommandGroups;
    elevator = elevatorCG.getElevator();
    coralRoller = elevatorCG.getCoralRoller();
    algaeRoller = elevatorCG.getAlgaeRoller();
  }

  AutoRoutine blueL4AlgAutoRoutine = super.getAutoFactory().newRoutine("BlueL4AlgaeRoutine");

  AutoTrajectory blueCenterToL4G = blueL4AlgAutoRoutine.trajectory("centerToL4G");
  AutoTrajectory blueL4GToAlgae = blueL4AlgAutoRoutine.trajectory("L4GToAlgae");
  AutoTrajectory blueAlgaeToProcess = blueL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  AutoTrajectory blueProcessToSource = blueL4AlgAutoRoutine.trajectory("ProcessToSource");

  @Override
  public String getName() {
    return "BlueL4AlgaeAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    blueL4AlgAutoRoutine
        .active()
        .onTrue(Commands.parallel(blueCenterToL4G.cmd(), elevatorCG.coralL4PositionCommand()));

    blueCenterToL4G
        .done()
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                blueL4GToAlgae.cmd()));

    blueL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevatorCG.algaeL3IntakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    blueAlgaeToProcess.cmd(), elevatorCG.algaeProcessorPositionCommand())));

    blueAlgaeToProcess
        .done()
        .onTrue(
            new SequentialCommandGroup(
                algaeRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    blueProcessToSource.cmd(),
                    elevator.createSetHeightCommand(ElevatorState.Intake))));

    return blueL4AlgAutoRoutine;
  }
}
