package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterState;
import frc.robot.elevator.Lifter;
import java.util.Optional;

public class BlueL4AlgaeAuto extends AutoMode {

  Lifter lifter;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  Elevator elevator;

  public BlueL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
    algaeRoller = elevator.getAlgaeRoller();
  }

  AutoRoutine blueL4AlgAutoRoutine = super.getAutoFactory().newRoutine("BlueL4AlgaeRoutine");

  AutoTrajectory blueCenterToL4G = blueL4AlgAutoRoutine.trajectory("blueCenterToL4G");
  // AutoTrajectory blueL4GToAlgae = blueL4AlgAutoRoutine.trajectory("L4GToAlgae");
  // AutoTrajectory blueAlgaeToProcess = blueL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  // AutoTrajectory blueProcessToSource = blueL4AlgAutoRoutine.trajectory("ProcessToSource");
  AutoTrajectory blueL4GBack = blueL4AlgAutoRoutine.trajectory("blueL4GBack");

  @Override
  public String getName() {
    return "BlueL4AlgaeAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    blueL4AlgAutoRoutine.active().onTrue(
      Commands.parallel(
        blueCenterToL4G.cmd(),
        Commands.sequence(
          Commands.waitSeconds(1.0),
          elevator.coralL4PositionCG())));

    blueCenterToL4G.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.waitSeconds(0.2),
            blueL4GBack.cmd()));

    blueL4GBack.done().onTrue(
        elevator.coralIntakeCG());

    // blueL4GToAlgae.done().onTrue(
    //     Commands.sequence(
    //         elevator.algaeL3IntakeCG().withTimeout(0.2),
    //         Commands.parallel(
    //             blueAlgaeToProcess.cmd(),
    //             elevator.algaeProcessorPositionCG())));

    // blueAlgaeToProcess.done().onTrue(
    //     Commands.sequence(
    //         algaeRoller.createOuttakeCommand().withTimeout(0.2),
    //         Commands.parallel(
    //             blueProcessToSource.cmd(),
    //             lifter.createSetHeightCommand(LifterState.CoralIntake))));
    // spotless:on

    return blueL4AlgAutoRoutine;
  }
}
