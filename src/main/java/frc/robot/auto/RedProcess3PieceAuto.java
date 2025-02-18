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

public class RedProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralIntake coralIntake;
  AlgaeIntake algaeIntake;

  public RedProcess3PieceAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralIntake coralIntakeSubsystem,
      AlgaeIntake algaeIntakeSubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
  }

  AutoRoutine redProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("redProcess3PieceRoutine");

  AutoTrajectory redCenterToL4F = redProcess3PieceRoutine.trajectory("redCenterToL4F");
  AutoTrajectory redL4FToSource = redProcess3PieceRoutine.trajectory("redL4FToSource");
  AutoTrajectory redSourceToL4D = redProcess3PieceRoutine.trajectory("redSourceToL4D");
  AutoTrajectory redL4DToSource = redProcess3PieceRoutine.trajectory("redL4DToSource");
  AutoTrajectory redSourceToL4C = redProcess3PieceRoutine.trajectory("redSourceToL4C");

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
    return "RedProcess3PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redCenterToL4F.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    redProcess3PieceRoutine
        .active()
        .onTrue(Commands.parallel(redCenterToL4F.cmd(), coralL4PositionCommand));

    redCenterToL4F
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5.0),
                new WaitCommand(0.2),
                coralIntake.createSetIntakeVelocityCommand(0),
                new ParallelCommandGroup(redL4FToSource.cmd(), coralIntakePositionCommand)));

    redL4FToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(redSourceToL4D.cmd(), coralL4PositionCommand))));

    redSourceToL4D
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5),
                new WaitCommand(0.2),
                coralIntake.createSetIntakeVelocityCommand(0),
                Commands.parallel(coralIntakePositionCommand, redL4DToSource.cmd())));

    redL4DToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(coralL4PositionCommand, redSourceToL4C.cmd()))));

    redSourceToL4C
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1), coralIntake.createSetIntakeVelocityCommand(-5)));

    return redProcess3PieceRoutine;
  }
}
