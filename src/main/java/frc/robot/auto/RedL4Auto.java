package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Lifter;
import java.util.Optional;

public class RedL4Auto extends AutoMode {

  Lifter lifter;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  Elevator elevator;

  public RedL4Auto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
    algaeRoller = elevator.getAlgaeRoller();
  }

  AutoRoutine redL4AutoRoutine = super.getAutoFactory().newRoutine("redL4Routine");

  AutoTrajectory redCenterToL4G = redL4AutoRoutine.trajectory("redCenterToL4G");
  AutoTrajectory redL4GBack = redL4AutoRoutine.trajectory("redL4GBack");

  @Override
  public String getName() {
    return "redL4Auto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    redL4AutoRoutine.active()
        .onTrue(redCenterToL4G.cmd())
        .onTrue(elevator.coralL4PositionCG());

    redCenterToL4G.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(1.0),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.waitSeconds(0.2)));

    // spotless:on

    return redL4AutoRoutine;
  }
}
