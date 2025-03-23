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

public class Red2PieceAuto extends AutoMode {
    
    Lifter lifter;
  CoralRoller coralRoller;
  Elevator elevator;

  public Red2PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    lifter = elevator.getLifter();
    coralRoller = elevator.getCoralRoller();
  }

  AutoRoutine red2PieceAutoRoutine = super.getAutoFactory().newRoutine("red2PieceRoutine");

  AutoTrajectory redLeftToL4F = red2PieceAutoRoutine.trajectory("redLeftToL4F");
  AutoTrajectory redL4FToSource = red2PieceAutoRoutine.trajectory("redL4FToSource");
  AutoTrajectory redSourceToL4C = red2PieceAutoRoutine.trajectory("redSourceToL4C");
  

  @Override
  public String getName() {
    return "red2PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redLeftToL4F.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    red2PieceAutoRoutine.active().onTrue(
      Commands.parallel(
        redLeftToL4F.cmd(),
        elevator.coralL4PositionCG().withTimeout(2.0)));
        

    redLeftToL4F.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1), 
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.parallel(
                redL4FToSource.cmd(),
                elevator.coralIntakeCG().withTimeout(2.0))));

    redL4FToSource.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(1.0),
            coralRoller.createIntakeCommand().withTimeout(1.0),
            Commands.parallel(
                redSourceToL4C.cmd(),
                elevator.coralL4PositionCG().withTimeout(2.0))));
    
    redSourceToL4C.done().onTrue(
        Commands.sequence(
            Commands.waitSeconds(0.1),
            coralRoller.createOuttakeCommand().withTimeout(0.2),
            Commands.waitSeconds(0.2)));

    // spotless:on

    return red2PieceAutoRoutine;
  }
}


