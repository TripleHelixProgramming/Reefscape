package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;

import java.util.Optional;

public class BlueProcess3PieceAuto2 extends AutoMode {

  CoralRoller coralRoller;
  Elevator elevator;

  public BlueProcess3PieceAuto2(Drivetrain drivetrain, Elevator elevatorSystem) {
    super(drivetrain);
    elevator = elevatorSystem;
    coralRoller = elevator.getCoralRoller();
  }

  AutoRoutine blueProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueProcess3PieceRoutine");

  AutoTrajectory blueCenterToL4F = blueProcess3PieceRoutine.trajectory("blueCenterToL4F");
  AutoTrajectory blueL4FToSource = blueProcess3PieceRoutine.trajectory("blueL4FToSource");
  AutoTrajectory blueSourceToL4D = blueProcess3PieceRoutine.trajectory("blueSourceToL4D");
  AutoTrajectory blueL4DToSource = blueProcess3PieceRoutine.trajectory("blueL4DToSource");
  AutoTrajectory blueSourceToL4C = blueProcess3PieceRoutine.trajectory("blueSourceToL4C");

  @Override
  public String getName() {
    return "BlueProcess3PieceAuto2";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4F.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    blueProcess3PieceRoutine.active()
        .onTrue(blueCenterToL4F.cmd());

    /*
     * 0 + 3 : At the dogleg, on final approach.
     * */
    blueCenterToL4F.atTime(3.0)
        .onTrue(elevator.coralL4PositionCG());

    /*
     * L4F + 0.1 : Run coral intake briefly.
     */
    blueCenterToL4F.doneDelayed(0.1)
        .onTrue(coralRoller.intake().withTimeout(0.2));

    /*
     * L4F + 0.4 : Outtake coral to L4.
     */
    blueCenterToL4F.doneDelayed(0.4)
        .onTrue(coralRoller.outtakeToL4().withTimeout(0.2));

    /*
     * L4F + 0.6 : Head to source.
     */
    blueCenterToL4F.doneDelayed(0.6)
        .onTrue(blueL4FToSource.cmd());

    /*
     * Source : Grab some coral and then head to L4D.
     */
    blueL4FToSource.done()
        .onTrue(
          coralRoller.intake()
            .until(coralRoller.hasCoral)
            .andThen(blueSourceToL4D.cmd()));

    /*
     * 0 + 3.2 : Raise elevator
     */
    blueSourceToL4D.atTime(3.2)
        .onTrue(elevator.coralL4PositionCG());

    /*
     * L4D + 0.1 : Run coral intake briefly.
     */
    blueSourceToL4D.doneDelayed(0.1)
        .onTrue(coralRoller.intake().withTimeout(0.2));

    /*
     * L4D + 0.4 : Outtake coral to L4.
     */
    blueSourceToL4D.doneDelayed(0.4)
        .onTrue(coralRoller.outtakeToL4().withTimeout(0.2));

    /*
     * L4D + 0.6 : Head to source.
     */
    blueSourceToL4D.doneDelayed(0.6)
        .onTrue(blueL4DToSource.cmd());

    /*
     * Source : Grab some coral and then head to L4C.
     */
    blueSourceToL4D.done()
      .onTrue(
        coralRoller.intake()
          .until(coralRoller.hasCoral)
          .andThen(blueSourceToL4C.cmd()));

    /*
     * 0 + 2.8 : Raise elevator
     */
    blueSourceToL4C.atTime(3.2)
        .onTrue(elevator.coralL4PositionCG());

    /*
     * L4C + 0.1 : Run coral intake briefly.
     */
    blueSourceToL4C.doneDelayed(0.1)
        .onTrue(coralRoller.intake().withTimeout(0.2));

    /*
     * L4C + 0.4 : Outtake coral to L4.
     */
    blueSourceToL4C.doneDelayed(0.4)
        .onTrue(coralRoller.outtakeToL4().withTimeout(0.2));

    // spotless:on

    return blueProcess3PieceRoutine;
  }
}
