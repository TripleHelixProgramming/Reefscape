package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeIntake;
import frc.robot.grippers.CoralRoller;
import java.util.Optional;

public class BlueProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeIntake algaeIntake;
  AutoCGs autoCG;

  public BlueProcess3PieceAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralRoller coralRollerSubsystem,
      AlgaeIntake algaeIntakeSubsystem,
      AutoCGs autoCommandGroups) {
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
                coralRoller.createSetIntakeVelocityCommand(-5.0),
                new WaitCommand(0.2),
                coralRoller.createSetIntakeVelocityCommand(0),
                new ParallelCommandGroup(
                    blueL4FToSource.cmd(), autoCG.coralIntakePositionCommand())));

    blueL4FToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralRoller.createSetIntakeVelocityCommand(5).until(coralRoller.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(blueSourceToL4D.cmd(), autoCG.coralL4PositionCommand()))));

    blueSourceToL4D
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralRoller.createSetIntakeVelocityCommand(-5),
                new WaitCommand(0.2),
                coralRoller.createSetIntakeVelocityCommand(0),
                Commands.parallel(autoCG.coralIntakePositionCommand(), blueL4DToSource.cmd())));

    blueL4DToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralRoller.createSetIntakeVelocityCommand(5).until(coralRoller.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(autoCG.coralL4PositionCommand(), blueSourceToL4C.cmd()))));

    blueSourceToL4C
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1), coralRoller.createSetIntakeVelocityCommand(-5)));

    return blueProcess3PieceRoutine;
  }
}
