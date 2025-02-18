package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeRoller;
import frc.robot.grippers.CoralRoller;
import java.util.Optional;

public class BlueL4AlgaeAuto extends AutoMode {

  Elevator elevator;
  CoralRoller coralRoller;
  AlgaeRoller algaeIntake;
  AutoCGs autoCG;

  public BlueL4AlgaeAuto(
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

  AutoRoutine blueL4AlgAutoRoutine = super.getAutoFactory().newRoutine("BlueL4AlgaeRoutine");

  AutoTrajectory blueCenterToL4G = blueL4AlgAutoRoutine.trajectory("centerToL4G");
  AutoTrajectory blueL4GToAlgae = blueL4AlgAutoRoutine.trajectory("L4GToAlgae");
  AutoTrajectory blueAlgaeToProcess = blueL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  AutoTrajectory blueProcessToSource = blueL4AlgAutoRoutine.trajectory("ProcessToSource");

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

    blueL4AlgAutoRoutine
        .active()
        .onTrue(Commands.parallel(blueCenterToL4G.cmd(), autoCG.coralL4PositionCommand()));

    blueCenterToL4G
        .done()
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                coralRoller.createSetIntakeVelocityCommand(-5.0),
                new WaitCommand(0.2),
                blueL4GToAlgae.cmd()));

    blueL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                autoCG.algaeL3PositionCommand(),
                algaeIntake.createSetIntakeVelocityCommand(5),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    blueAlgaeToProcess.cmd(), autoCG.algaeProcessorPositionCommand())));

    blueAlgaeToProcess
        .done()
        .onTrue(
            new SequentialCommandGroup(
                algaeIntake.createSetIntakeVelocityCommand(-5),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    blueProcessToSource.cmd(),
                    elevator.createSetPositionCommand(ElevatorState.Intake))));

    return blueL4AlgAutoRoutine;
  }
}
