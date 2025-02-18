package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterState;
import frc.robot.elevator.Lifter;
import java.util.Optional;

public class BlueL4AlgaeAuto extends AutoMode {

  Lifter lifter;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  Elevator elevator;

  public BlueL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
    algaeRoller = elevator.getAlgaeRoller();
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
        .onTrue(Commands.parallel(blueCenterToL4G.cmd(), elevator.coralL4PositionCG()));

    blueCenterToL4G
        .done()
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                coralRoller.createOuttakeCommand(),
                new WaitCommand(0.2),
                blueL4GToAlgae.cmd()));

    blueL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevator.algaeL3IntakeCG(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    blueAlgaeToProcess.cmd(), elevator.algaeProcessorPositionCG())));

    blueAlgaeToProcess
        .done()
        .onTrue(
            new SequentialCommandGroup(
                algaeRoller.createOuttakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    blueProcessToSource.cmd(), lifter.createSetHeightCommand(LifterState.Intake))));

    return blueL4AlgAutoRoutine;
  }
}
