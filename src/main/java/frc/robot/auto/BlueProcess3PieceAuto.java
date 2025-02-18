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

public class BlueProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeRoller algaeIntake;
  ElevatorCGs autoCG;

  public BlueProcess3PieceAuto(
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

  AutoRoutine blueProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueProcess3PieceRoutine");

  AutoTrajectory blueCenterToL4F = blueProcess3PieceRoutine.trajectory("blueCenterToL4F");
  AutoTrajectory blueL4FToSource = blueProcess3PieceRoutine.trajectory("blueL4FToSource");
  AutoTrajectory blueSourceToL4D = blueProcess3PieceRoutine.trajectory("blueSourceToL4D");
  AutoTrajectory blueL4DToSource = blueProcess3PieceRoutine.trajectory("blueL4DToSource");
  AutoTrajectory blueSourceToL4C = blueProcess3PieceRoutine.trajectory("blueSourceToL4C");

  @Override
  public String getName() {
    return "BlueProcess3PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4F.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    blueProcess3PieceRoutine
        .active()
        .onTrue(Commands.parallel(blueCenterToL4F.cmd(), autoCG.coralL4PositionCommand()));

    blueCenterToL4F
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                coralRoller.createStopCommand(),
                new ParallelCommandGroup(blueL4FToSource.cmd(), autoCG.coralIntakeCommand())));

    blueL4FToSource
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.2),
                Commands.parallel(blueSourceToL4D.cmd(), autoCG.coralL4PositionCommand())));

    blueSourceToL4D
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                coralRoller.createStopCommand(),
                Commands.parallel(autoCG.coralIntakeCommand(), blueL4DToSource.cmd())));

    blueL4DToSource
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.2),
                Commands.parallel(autoCG.coralL4PositionCommand(), blueSourceToL4C.cmd())));

    blueSourceToL4C
        .done()
        .onTrue(Commands.sequence(new WaitCommand(0.1), coralRoller.createSetOuttakeCommand()));

    return blueProcess3PieceRoutine;
  }
}
