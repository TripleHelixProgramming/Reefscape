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

public class RedNoProcess3PieceAuto extends AutoMode {

  Elevator elevator;
  CoralIntake coralIntake;
  AlgaeIntake algaeIntake;

  public RedNoProcess3PieceAuto(
      Drivetrain drivetrain,
      Elevator elevatorsubsystem,
      CoralIntake coralIntakeSubsystem,
      AlgaeIntake algaeIntakeSubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
  }

  AutoRoutine redNoProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("redNoProcess3PieceRoutine");

  AutoTrajectory redNoProcessToL4I = redNoProcess3PieceRoutine.trajectory("redNoProcessToL4I");
  AutoTrajectory redL4IToSource = redNoProcess3PieceRoutine.trajectory("redL4IToSource");
  AutoTrajectory redSourceToL4L = redNoProcess3PieceRoutine.trajectory("redSourceToL4L");
  AutoTrajectory redL4LToSource = redNoProcess3PieceRoutine.trajectory("redL4LToSource");
  AutoTrajectory redSourceToL4K = redNoProcess3PieceRoutine.trajectory("redSourceToL4K");

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
    return "RedNoProcess3PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redNoProcessToL4I.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    redNoProcess3PieceRoutine
        .active()
        .onTrue(Commands.parallel(redNoProcessToL4I.cmd(), coralL4PositionCommand));

    redNoProcessToL4I
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5.0),
                new WaitCommand(0.2),
                coralIntake.createSetIntakeVelocityCommand(0),
                new ParallelCommandGroup(redL4IToSource.cmd(), coralIntakePositionCommand)));

    redL4IToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(redSourceToL4L.cmd(), coralL4PositionCommand))));

    redSourceToL4L
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1),
                coralIntake.createSetIntakeVelocityCommand(-5),
                new WaitCommand(0.2),
                coralIntake.createSetIntakeVelocityCommand(0),
                Commands.parallel(coralIntakePositionCommand, redL4LToSource.cmd())));

    redL4LToSource
        .done()
        .onTrue(
            Commands.parallel(
                coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                Commands.sequence(
                    new WaitCommand(0.2),
                    Commands.parallel(coralL4PositionCommand, redSourceToL4K.cmd()))));

    redSourceToL4K
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(0.1), coralIntake.createSetIntakeVelocityCommand(-5)));

    return redNoProcess3PieceRoutine;
  }
}
