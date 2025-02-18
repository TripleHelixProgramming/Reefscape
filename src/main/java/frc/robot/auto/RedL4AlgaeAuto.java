package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterState;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Lifter;
import java.util.Optional;

public class RedL4AlgaeAuto extends AutoMode {

  Lifter lifter;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  Elevator elevator;

  public RedL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
    algaeRoller = elevator.getAlgaeRoller();
  }

  AutoRoutine redL4AlgAutoRoutine = super.getAutoFactory().newRoutine("redL4AlgaeRoutine");

  AutoTrajectory redCenterToL4G = redL4AlgAutoRoutine.trajectory("centerToL4G");
  AutoTrajectory redL4GToAlgae = redL4AlgAutoRoutine.trajectory("L4GToAlgae");
  AutoTrajectory redAlgaeToProcess = redL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  AutoTrajectory redProcessToSource = redL4AlgAutoRoutine.trajectory("ProcessToSource");

  @Override
  public String getName() {
    return "redL4AlgaeAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    redL4AlgAutoRoutine
        .active()
        .onTrue(Commands.parallel(redCenterToL4G.cmd(), elevator.coralL4PositionCommand()));

    redCenterToL4G
        .done()
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                coralRoller.createOuttakeCommand(),
                new WaitCommand(0.2),
                redL4GToAlgae.cmd()));

    redL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevator.algaeL3IntakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    redAlgaeToProcess.cmd(), elevator.algaeProcessorPositionCommand())));

    redAlgaeToProcess
        .done()
        .onTrue(
            new SequentialCommandGroup(
                algaeRoller.createOuttakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    redProcessToSource.cmd(), lifter.createSetHeightCommand(LifterState.Intake))));

    return redL4AlgAutoRoutine;
  }
}
