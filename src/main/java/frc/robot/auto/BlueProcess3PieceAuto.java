package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeIntakeConstants.AlgaeIntakeStates;
import frc.robot.Constants.CoralIntakeConstants.CoralIntakeStates;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeIntake;
import frc.robot.grippers.CoralIntake;
import java.util.Optional;

public class BlueProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralIntake coralIntake;
  AlgaeIntake algaeIntake;

  public BlueProcess3PieceAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralIntake coralIntakeSubsystem,
      AlgaeIntake algaeIntakeSubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
  }

  AutoRoutine blueProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueProcess3PieceRoutine");

  AutoTrajectory blueCenterToL4F = blueProcess3PieceRoutine.trajectory("blueCenterToL4F");
  AutoTrajectory blueL4FToSource = blueProcess3PieceRoutine.trajectory("blueL4FToSource");
  AutoTrajectory blueSourceToL4D = blueProcess3PieceRoutine.trajectory("blueSourceToL4D");
  AutoTrajectory blueL4DToSource = blueProcess3PieceRoutine.trajectory("blueL4DToSource");
  AutoTrajectory blueSourceToL4C = blueProcess3PieceRoutine.trajectory("blueSourceToL4C");

  private ParallelCommandGroup coralL4PositionCommand =
      new ParallelCommandGroup(
          elevator.createSetPositionCommand(ElevatorState.L4),
          coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L4.angle),
          algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));

  private ParallelCommandGroup coralIntakePositionCommand =
      new ParallelCommandGroup(
          elevator.createSetPositionCommand(ElevatorState.Intake),
          coralIntake.createSetRotationPositionCommand(CoralIntakeStates.Intake.angle),
          algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle));

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
        .onTrue(Commands.parallel(blueCenterToL4F.cmd(), coralL4PositionCommand));

    blueCenterToL4F
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5.0),
                new WaitCommand(0.2),
                coralIntake.createSetIntakeVelocityCommand(0),
                new ParallelCommandGroup(blueL4FToSource.cmd(), coralIntakePositionCommand)));

    blueL4FToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(blueSourceToL4D.cmd(), coralL4PositionCommand))));

    blueSourceToL4D
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5),
                new WaitCommand(0.2),
                coralIntake.createSetIntakeVelocityCommand(0),
                Commands.parallel(coralIntakePositionCommand, blueL4DToSource.cmd())));

    blueL4DToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(coralL4PositionCommand, blueSourceToL4C.cmd()))));

    blueSourceToL4C
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1), coralIntake.createSetIntakeVelocityCommand(-5)));

    return blueProcess3PieceRoutine;
  }
}
