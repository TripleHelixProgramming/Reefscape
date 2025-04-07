package frc.robot.auto;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;

public abstract class L4MultiAuto extends AutoMode {
  protected final String name;
  protected final CoralRoller coralRoller;
  protected final Elevator elevator;

  public L4MultiAuto(String name, Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    this.name = name;
    elevator = elevatorSystem;
    coralRoller = elevator.getCoralRoller();
  }

  @Override
  public String getName() {
    return name;
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
    scoreTrajectory.atTime(finalApprochTime).onTrue(coralRoller.jiggle());
    scoreTrajectory.doneDelayed(0.2).onTrue(coralRoller.outtakeToL4().withTimeout(0.2));
    scoreTrajectory.doneDelayed(0.4).onTrue(elevator.coralIntakeCG());
    scoreTrajectory.doneDelayed(0.4).onTrue(nextAction);
  }

  /**
   * Grabs some coral once the provided trajectory finishes.
   *
   * @param sourceTrajectory The active trajectory leading to the coral source
   * @param nextAction an action to perform after the coral is grabbed
   */
  protected void grabSomeCoralThen(AutoTrajectory sourceTrajectory, Command nextAction) {
    sourceTrajectory.done().onTrue(coralRoller.intake().until(coralRoller.hasCoral));
    sourceTrajectory.doneDelayed(1.0).onTrue(nextAction);
  }
}
