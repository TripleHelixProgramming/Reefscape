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

public class RedL4AlgaeAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  ElevatorCGs elevatorCG;

  public RedL4AlgaeAuto(Drivetrain drivetrain, ElevatorCGs autoCommandGroups) {
    super(drivetrain);
    elevatorCG = autoCommandGroups;
    elevator = elevatorCG.getElevator();
    coralRoller = elevatorCG.getCoralRoller();
    algaeRoller = elevatorCG.getAlgaeRoller();
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
        .onTrue(Commands.parallel(redCenterToL4G.cmd(), elevatorCG.coralL4PositionCommand()));

    redCenterToL4G
        .done()
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                redL4GToAlgae.cmd()));

    redL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevatorCG.algaeL3IntakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    redAlgaeToProcess.cmd(), elevatorCG.algaeProcessorPositionCommand())));

    redAlgaeToProcess
        .done()
        .onTrue(
            new SequentialCommandGroup(
                algaeRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    redProcessToSource.cmd(),
                    elevator.createSetHeightCommand(ElevatorState.Intake))));

    return redL4AlgAutoRoutine;
  }
}
