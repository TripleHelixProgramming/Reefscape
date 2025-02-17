package frc.robot.auto;

import java.util.Optional;

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

public class BlueNoProcess3PieceAuto extends AutoMode {

    Elevator elevator;
    CoralIntake coralIntake;
    AlgaeIntake algaeIntake;

    public BlueNoProcess3PieceAuto(
        Drivetrain drivetrain,
        Elevator elevatorsubsystem,
        CoralIntake coralIntakeSubsystem,
        AlgaeIntake algaeIntakeSubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
    coralIntake = coralIntakeSubsystem;
    algaeIntake = algaeIntakeSubsystem;
    }

    AutoRoutine blueNoProcess3PieceRoutine = super.getAutoFactory().newRoutine("blueNoProcess3PieceRoutine");

    AutoTrajectory blueNoProcessToL4I = blueNoProcess3PieceRoutine.trajectory("blueNoProcessToL4I");
    AutoTrajectory blueL4IToSource = blueNoProcess3PieceRoutine.trajectory("blueL4IToSource");
    AutoTrajectory blueSourceToL4L = blueNoProcess3PieceRoutine.trajectory("blueSourceToL4L");
    AutoTrajectory blueL4LToSource = blueNoProcess3PieceRoutine.trajectory("blueL4LToSource");
    AutoTrajectory blueSourceToL4K = blueNoProcess3PieceRoutine.trajectory("blueSourceToL4K");

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
        return "BlueNoProcess3PieceAuto";
    }

    @Override
    public Optional<Pose2d> getInitialPose() {
        return blueNoProcessToL4I.getInitialPose();
    }

    @Override
    public AutoRoutine getAutoRoutine() {

        blueNoProcess3PieceRoutine
            .active()
            .onTrue(
                Commands.parallel(
                    blueNoProcessToL4I.cmd(),
                    coralL4PositionCommand));

        blueNoProcessToL4I
            .done()
            .onTrue(
                Commands.sequence(
                    new WaitCommand(0.1),
                    coralIntake.createSetIntakeVelocityCommand(-5.0),
                    new WaitCommand(0.2),
                    coralIntake.createSetIntakeVelocityCommand(0),
                    new ParallelCommandGroup(
                        blueL4IToSource.cmd(),
                        coralIntakePositionCommand)));

        blueL4IToSource
            .done()
            .onTrue(
                Commands.parallel(
                    coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                    Commands.sequence(
                        new WaitCommand(0.2),
                        Commands.parallel(
                            blueSourceToL4L.cmd(),
                            coralL4PositionCommand))));

        blueSourceToL4L
            .done()
            .onTrue(
                Commands.sequence(
                    new WaitCommand(0.1),
                    coralIntake.createSetIntakeVelocityCommand(-5),
                    new WaitCommand(0.2),
                    coralIntake.createSetIntakeVelocityCommand(0),
                    Commands.parallel(
                        coralIntakePositionCommand,
                        blueL4LToSource.cmd())));

        blueL4LToSource
            .done()
            .onTrue(
                Commands.parallel(
                    coralIntake.createSetIntakeVelocityCommand(5).until(coralIntake.hasCoralPiece()),
                    Commands.sequence(
                        new WaitCommand(0.2),
                        Commands.parallel(
                            coralL4PositionCommand,
                            blueSourceToL4K.cmd()))));

        blueSourceToL4K
            .done()
            .onTrue(
                Commands.sequence(
                    new WaitCommand(0.1),
                    coralIntake.createSetIntakeVelocityCommand(-5)));

        return blueNoProcess3PieceRoutine;
    }
}