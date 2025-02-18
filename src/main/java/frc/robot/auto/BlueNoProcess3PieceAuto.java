package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class BlueNoProcess3PieceAuto extends AutoMode {

  CoralRoller coralRoller;
  Elevator elevator;

  public BlueNoProcess3PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    coralRoller = elevator.getCoralRoller();
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

    // spotless:off
    blueNoProcess3PieceRoutine.active().onTrue(
        Commands.parallel(
            blueNoProcessToL4I.cmd(),
            elevator.coralL4PositionCG()));

    blueNoProcessToL4I.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.parallel(
                blueL4IToSource.cmd(),
                elevator.coralIntakeCG())));

    blueL4IToSource.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.2),
            Commands.parallel(
                blueSourceToL4L.cmd(),
                elevator.coralL4PositionCG())));

    blueSourceToL4L.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.parallel(
                elevator.coralIntakeCG(),
                blueL4LToSource.cmd())));

    blueL4LToSource.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.2),
            Commands.parallel(
                elevator.coralL4PositionCG(),
                blueSourceToL4K.cmd())));

    blueSourceToL4K.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand()));
    // spotless:on

    return blueNoProcess3PieceRoutine;
  }
}
