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

public class BlueProcess2PieceAuto extends AutoMode {

  CoralRoller coralRoller;
  Elevator elevator;

  public BlueProcess2PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    coralRoller = elevator.getCoralRoller();
  }

  AutoRoutine blueProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueProcess2PieceRoutine");

  AutoTrajectory blueCenterToL4F = blueProcess3PieceRoutine.trajectory("blueCenterToL4F");
  AutoTrajectory blueL4FToSource = blueProcess3PieceRoutine.trajectory("blueL4FToSource");
  AutoTrajectory blueSourceToL4D = blueProcess3PieceRoutine.trajectory("blueSourceToL4D");

  @Override
  public String getName() {
    return "BlueProcess2PieceAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4F.getInitialPose();
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
    scoreTrajectory.doneDelayed(0.1).onTrue(coralRoller.outtakeToL4().withTimeout(0.2));
    scoreTrajectory.doneDelayed(0.2).onTrue(elevator.coralIntakeCG());
    scoreTrajectory.doneDelayed(0.2).onTrue(nextAction);
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
        .doneDelayed(1.0)
        .onTrue(nextAction);
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // Score at L4F
    blueProcess3PieceRoutine.active().onTrue(blueCenterToL4F.cmd());
    scoreToL4Then(blueCenterToL4F, 1.0, blueL4FToSource.cmd());

    // Grab some coral then score at L4D
    grabSomeCoralThen(blueL4FToSource, blueSourceToL4D.cmd());
    scoreToL4Then(blueSourceToL4D, 1.0, Commands.none());

    return blueProcess3PieceRoutine;
  }
}
