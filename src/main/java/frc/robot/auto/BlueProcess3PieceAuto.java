package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class BlueProcess3PieceAuto extends AutoMode {

  CoralRoller coralRoller;
  Elevator elevator;

  public BlueProcess3PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    coralRoller = elevator.getCoralRoller();
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

    // spotless:off
    blueProcess3PieceRoutine.active().onTrue(blueCenterToL4F.cmd());

    blueCenterToL4F.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            elevator.coralL4PositionCG(),
            Commands.waitSeconds(0.1),
            coralRoller.outtakeToL4().withTimeout(0.2),
            Commands.parallel(
                blueL4FToSource.cmd(),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    elevator.coralIntakeCG()))));

    blueL4FToSource.done().onTrue(
        Commands.sequence(
            coralRoller.intake().withTimeout(0.2),
            blueSourceToL4D.cmd()));

    blueSourceToL4D.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            elevator.coralL4PositionCG(),
            Commands.waitSeconds(0.1),
            coralRoller.outtakeToL4().withTimeout(0.2),
            Commands.parallel(
                blueL4DToSource.cmd(),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    elevator.coralIntakeCG()
                ))));

    blueL4DToSource.done().onTrue(
        Commands.sequence(
            coralRoller.intake().withTimeout(0.2),
            elevator.coralL4PositionCG()));

    blueSourceToL4C.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            elevator.coralL4PositionCG(),
            Commands.waitSeconds(0.1),
            coralRoller.outtakeToL4()));
    // spotless:on

    return blueProcess3PieceRoutine;
  }
}
