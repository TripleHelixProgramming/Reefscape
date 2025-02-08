package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeIntake;
import frc.robot.grippers.CoralIntake;

import java.util.Optional;

public class BlueL4AlgaeAuto extends AutoMode {

  Elevator elevator;
  CoralIntake coralIntake;
  AlgaeIntake algaeIntake;

  public BlueL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorsubsystem, CoralIntake coralIntakeSubsystem, AlgaeIntake algaeIntakeSubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
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
        .onTrue(
            Commands.parallel(
                blueCenterToL4G.cmd(), elevator.createSetPositionCommand(ElevatorPosition.L4)));

    blueCenterToL4G
        .done()
        .onTrue(new SequentialCommandGroup(new WaitCommand(0.1), coralIntake.createSetIntakeVelocityCommand(-5.0), new WaitCommand(0.2), blueL4GToAlgae.cmd()));

    blueL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevator.createSetPositionCommand(
                    ElevatorPosition.L3), 
                algaeIntake.createSetIntakeVelocityCommand(5),
                new WaitCommand(0.2),
                blueAlgaeToProcess.cmd()));

    blueAlgaeToProcess
        .done()
        .onTrue(new SequentialCommandGroup(algaeIntake.createSetIntakeVelocityCommand(-5), new WaitCommand(0.2), blueProcessToSource.cmd()));

    return blueL4AlgAutoRoutine;
  }
}
