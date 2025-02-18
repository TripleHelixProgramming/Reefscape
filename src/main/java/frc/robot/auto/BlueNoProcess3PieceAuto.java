package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.ElevatorCGs;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;

import java.util.Optional;

public class BlueNoProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeRoller algaeIntake;
  ElevatorCGs autoCG;

  public BlueNoProcess3PieceAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralRoller coralRollerSubsystem,
      AlgaeRoller algaeIntakeSubsystem,
      ElevatorCGs autoCommandGroups) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralRoller = coralRollerSubsystem;
    algaeIntake = algaeIntakeSubsystem;
    autoCG = autoCommandGroups;
  }

  AutoRoutine blueNoProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueNoProcess3PieceRoutine");

  AutoTrajectory blueNoProcessToL4I = blueNoProcess3PieceRoutine.trajectory("blueNoProcessToL4I");
  AutoTrajectory blueL4IToSource = blueNoProcess3PieceRoutine.trajectory("blueL4IToSource");
  AutoTrajectory blueSourceToL4L = blueNoProcess3PieceRoutine.trajectory("blueSourceToL4L");
  AutoTrajectory blueL4LToSource = blueNoProcess3PieceRoutine.trajectory("blueL4LToSource");
  AutoTrajectory blueSourceToL4K = blueNoProcess3PieceRoutine.trajectory("blueSourceToL4K");

  @Override
  public String getName() {
    return "BlueNoProcess3PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueNoProcessToL4I.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    blueNoProcess3PieceRoutine
        .active()
        .onTrue(Commands.parallel(blueNoProcessToL4I.cmd(), autoCG.coralL4PositionCommand()));

    blueNoProcessToL4I
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                coralRoller.createStopCommand(),
                new ParallelCommandGroup(blueL4IToSource.cmd(), autoCG.coralIntakeCommand())));

    blueL4IToSource
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.2),
                Commands.parallel(blueSourceToL4L.cmd(), autoCG.coralL4PositionCommand())));

    blueSourceToL4L
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                coralRoller.createStopCommand(),
                Commands.parallel(autoCG.coralIntakeCommand(), blueL4LToSource.cmd())));

    blueL4LToSource
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.2),
                Commands.parallel(autoCG.coralL4PositionCommand(), blueSourceToL4K.cmd())));

    blueSourceToL4K
        .done()
        .onTrue(Commands.sequence(new WaitCommand(0.1), coralRoller.createSetOuttakeCommand()));

    return blueNoProcess3PieceRoutine;
  }
}
