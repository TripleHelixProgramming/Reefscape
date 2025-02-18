package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeRoller;
import frc.robot.grippers.CoralRoller;
import java.util.Optional;

public class RedNoProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeRoller algaeIntake;
  AutoCGs autoCG;

  public RedNoProcess3PieceAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralRoller coralRollerSubsystem,
      AlgaeRoller algaeIntakeSubsystem,
      AutoCGs autoCommandGroups) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralRoller = coralRollerSubsystem;
    algaeIntake = algaeIntakeSubsystem;
    autoCG = autoCommandGroups;
  }

  AutoRoutine redNoProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("redNoProcess3PieceRoutine");

  AutoTrajectory redNoProcessToL4I = redNoProcess3PieceRoutine.trajectory("redNoProcessToL4I");
  AutoTrajectory redL4IToSource = redNoProcess3PieceRoutine.trajectory("redL4IToSource");
  AutoTrajectory redSourceToL4L = redNoProcess3PieceRoutine.trajectory("redSourceToL4L");
  AutoTrajectory redL4LToSource = redNoProcess3PieceRoutine.trajectory("redL4LToSource");
  AutoTrajectory redSourceToL4K = redNoProcess3PieceRoutine.trajectory("redSourceToL4K");

  @Override
  public String getName() {
    return "RedNoProcess3PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redNoProcessToL4I.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    redNoProcess3PieceRoutine
        .active()
        .onTrue(Commands.parallel(redNoProcessToL4I.cmd(), autoCG.coralL4PositionCommand()));

    redNoProcessToL4I
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                coralRoller.createStopCommand(),
                new ParallelCommandGroup(
                    redL4IToSource.cmd(), autoCG.coralIntakePositionCommand())));

    redL4IToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralRoller.createSetIntakeCommand().until(coralRoller.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(redSourceToL4L.cmd(), autoCG.coralL4PositionCommand()))));

    redSourceToL4L
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetOuttakeCommand(),
                new WaitCommand(0.2),
                coralRoller.createStopCommand(),
                Commands.parallel(autoCG.coralIntakePositionCommand(), redL4LToSource.cmd())));

    redL4LToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralRoller.createSetIntakeCommand().until(coralRoller.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(autoCG.coralL4PositionCommand(), redSourceToL4K.cmd()))));

    redSourceToL4K
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1), coralRoller.createSetOuttakeCommand()));

    return redNoProcess3PieceRoutine;
  }
}
