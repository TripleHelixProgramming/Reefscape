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

public class BlueL4Auto extends AutoMode {

  Lifter lifter;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  Elevator elevator;

  public BlueL4Auto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
    algaeRoller = elevator.getAlgaeRoller();
  }

  AutoRoutine blueL4AutoRoutine = super.getAutoFactory().newRoutine("BlueL4Routine");

  AutoTrajectory blueCenterToL4G = blueL4AutoRoutine.trajectory("blueCenterToL4G");
  AutoTrajectory blueL4GBack = blueL4AutoRoutine.trajectory("blueL4GBack");

  @Override
  public String getName() {
    return "BlueL4Auto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    blueL4AutoRoutine.active()
        .onTrue(blueCenterToL4G.cmd())
        .onTrue(elevator.coralL4PositionCG());

    blueCenterToL4G.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(1.0),
            coralRoller.outtakeToL4().withTimeout(0.2)));

    // spotless:on

    return blueL4AutoRoutine;
  }
}
