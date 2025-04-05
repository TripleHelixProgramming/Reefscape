package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class RedProcess2PieceAuto extends AutoMode {

  CoralRoller coralRoller;
  Elevator elevator;

  public RedProcess2PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    coralRoller = elevator.getCoralRoller();
  }

  AutoRoutine redProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("redProcess2PieceRoutine");

  AutoTrajectory redCenterToL4F = redProcess3PieceRoutine.trajectory("redCenterToL4F");
  AutoTrajectory redL4FToSource = redProcess3PieceRoutine.trajectory("redL4FToSource");
  AutoTrajectory redSourceToL4D = redProcess3PieceRoutine.trajectory("redSourceToL4D");

  @Override
  public String getName() {
    return "RedProcess2PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redCenterToL4F.getInitialPose();
  }

  /**
   * Attempts to score a coral at L4 height when the provided trajectory finishes.
   *
   * <p>As the robot approaches the scoring position, the elevator is raised. The coral is jiggled
   * slightly, and then ejected. After ejecting, the elevator is lowered back to coral intake height
   *
   * @param scoreTrajectory The active trajectory leading to the scoring position
   * @param finalApprochTime The time in the trajectory when the elevator should raise
   * @param nextAction an action to perform after the scoring is complete
   */
  protected void scoreToL4Then(
      AutoTrajectory scoreTrajectory, double finalApprochTime, Command nextAction) {

      scoreTrajectory.atTime(finalApprochTime).onTrue(elevator.coralL4PositionCG());
      // scoreTrajectory.doneDelayed(0.1).onTrue(coralRoller.intake().withTimeout(0.2));
      scoreTrajectory.doneDelayed(0.2).onTrue(coralRoller.outtakeToL4().withTimeout(0.2));
      scoreTrajectory.doneDelayed(0.5).onTrue(elevator.coralIntakeCG());
      scoreTrajectory.doneDelayed(0.5).onTrue(nextAction);
      }

  /**
   * Grabs some coral once the provided trajectory finishes.
   *
   * @param sourceTrajectory The active trajectory leading to the coral source
   * @param nextAction an action to perform after the coral is grabbed
   */
  protected void grabSomeCoralThen(AutoTrajectory sourceTrajectory, Command nextAction) {
    sourceTrajectory
        .done()
        .onTrue(coralRoller.intake().until(coralRoller.hasCoral));
    sourceTrajectory
        .done()
        .and(coralRoller.hasCoral)
        .onTrue(nextAction);
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // Score at L4F
    redProcess3PieceRoutine.active().onTrue(redCenterToL4F.cmd());
    scoreToL4Then(redCenterToL4F, 1.0, redL4FToSource.cmd());

    // Grab some coral then score at L4D
    grabSomeCoralThen(redL4FToSource, redSourceToL4D.cmd());
    scoreToL4Then(redSourceToL4D, 1.0, Commands.none());

    return redProcess3PieceRoutine;
  }
}
