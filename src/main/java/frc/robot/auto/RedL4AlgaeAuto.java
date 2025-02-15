package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeIntakeConstants.AlgaeIntakeStates;
import frc.robot.Constants.CoralIntakeConstants.CoralIntakeStates;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeIntake;
import frc.robot.grippers.CoralIntake;
import java.util.Optional;

public class RedL4AlgaeAuto extends AutoMode {

  Elevator elevator;
  CoralIntake coralIntake;
  AlgaeIntake algaeIntake;

  public RedL4AlgaeAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralIntake coralIntakeSubsystem,
      AlgaeIntake algaeIntakeSubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
  }

  AutoRoutine redL4AlgAutoRoutine = super.getAutoFactory().newRoutine("redL4AlgaeRoutine");

  AutoTrajectory redCenterToL4G = redL4AlgAutoRoutine.trajectory("centerToL4G");
  AutoTrajectory redL4GToAlgae = redL4AlgAutoRoutine.trajectory("L4GToAlgae");
  AutoTrajectory redAlgaeToProcess = redL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  AutoTrajectory redProcessToSource = redL4AlgAutoRoutine.trajectory("ProcessToSource");

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

    redL4AlgAutoRoutine
        .active()
        .onTrue(
            Commands.parallel(
                redCenterToL4G.cmd(), 
                elevator.createSetPositionCommand(ElevatorState.L4), 
                coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L4.angle), 
                algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle)));

    redCenterToL4G
        .done()
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5.0),
                new WaitCommand(0.2),
                redL4GToAlgae.cmd()));

    redL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                new ParallelCommandGroup(
                  elevator.createSetPositionCommand(ElevatorState.AlgaeL3),
                  coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
                  algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.L3.angle)),
                algaeIntake.createSetIntakeVelocityCommand(5),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                  redAlgaeToProcess.cmd(),
                  elevator.createSetPositionCommand(ElevatorState.Processor),
                  algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Processor.angle)
                )));

    redAlgaeToProcess
        .done()
        .onTrue(
            new SequentialCommandGroup(
                algaeIntake.createSetIntakeVelocityCommand(-5),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                  redProcessToSource.cmd(),
                  elevator.createSetPositionCommand(ElevatorState.Intake))));

    return redL4AlgAutoRoutine;
  }
}
