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

public class RedL4AlgaeAuto extends AutoMode {

  Lifter lifter;
  CoralRoller coralRoller;
  AlgaeRoller algaeRoller;
  Elevator elevator;

  public RedL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
    algaeRoller = elevator.getAlgaeRoller();
  }

  AutoRoutine redL4AlgAutoRoutine = super.getAutoFactory().newRoutine("redL4AlgaeRoutine");

  AutoTrajectory redCenterToL4G = redL4AlgAutoRoutine.trajectory("redCenterToL4G");
  // AutoTrajectory redL4GToAlgae = redL4AlgAutoRoutine.trajectory("L4GToAlgae");
  // AutoTrajectory redAlgaeToProcess = redL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  // AutoTrajectory redProcessToSource = redL4AlgAutoRoutine.trajectory("ProcessToSource");
  AutoTrajectory redL4GBack = redL4AlgAutoRoutine.trajectory("redL4GBack");

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

    // spotless:off
    redL4AlgAutoRoutine.active().onTrue(
        Commands.parallel(
            redCenterToL4G.cmd(),
            elevator.coralL4PositionCG()));

    redCenterToL4G.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            redL4GBack.cmd()));

    redL4GBack.done().onTrue(
        elevator.coralIntakeCG());

    // redL4GToAlgae.done().onTrue(
    //     Commands.sequence(
    //         elevator.algaeL3IntakeCG().withTimeout(0.2),
    //         Commands.parallel(
    //             redAlgaeToProcess.cmd(),
    //             elevator.algaeProcessorPositionCG())));

    // redAlgaeToProcess.done().onTrue(
    //     Commands.sequence(
    //         algaeRoller.createOuttakeCommand().withTimeout(0.2),
    //         Commands.parallel(
    //             redProcessToSource.cmd(),
    //             lifter.createSetHeightCommand(LifterState.CoralIntake))));
    // spotless:on

    return redL4AlgAutoRoutine;
  }
}
