package frc.robot.auto;

import java.util.Optional;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Lifter;

public class Blue2PieceAuto extends AutoMode{

Lifter lifter;
CoralRoller coralRoller;
Elevator elevator;
AlgaeRoller algaeRoller;

public Blue2PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
}

AutoRoutine blue2PieceAutoRoutine = super.getAutoFactory().newRoutine("blue2PieceRoutine");

AutoTrajectory blueLeftToL4F = blue2PieceAutoRoutine.trajectory("blueLeftToL4F");
AutoTrajectory blueL4FToSource = blue2PieceAutoRoutine.trajectory("blueL4FToSource");
AutoTrajectory blueSourceToL4C = blue2PieceAutoRoutine.trajectory("blueSourceToL4C");

@Override
public String getName() {
    return "blue2PieceAuto";
}

@Override
public Optional<Pose2d> getInitialPose(){
    return blueLeftToL4F.getInitialPose();
}

@Override
public AutoRoutine getAutoRoutine() {

     blue2PieceAutoRoutine.active().onTrue(
      Commands.parallel(
        blueLeftToL4F.cmd(),
        elevator.coralL4PositionCG().withTimeout(2.0)));
        

    blueLeftToL4F.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1), 
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.parallel(
                blueL4FToSource.cmd(),
                elevator.coralIntakeCG().withTimeout(2.0))));

    blueL4FToSource.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(1.0),
            coralRoller.createIntakeCommand().withTimeout(1.0),
            Commands.parallel(
                blueSourceToL4C.cmd(),
                elevator.coralL4PositionCG().withTimeout(2.0))));
    
    blueSourceToL4C.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.waitSeconds(0.2)));

    // spotless:on

    return blue2PieceAutoRoutine;
  }
}


