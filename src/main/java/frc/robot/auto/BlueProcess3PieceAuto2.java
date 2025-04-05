package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class BlueProcess3PieceAuto2 extends L4MultiAuto {

  CoralRoller coralRoller;
  Elevator elevator;

  public BlueProcess3PieceAuto2(Drivetrain drivetrain, Elevator elevatorSystem) {
    super("BlueProcess3PieceAuto2", drivetrain, elevatorSystem);
  }

  AutoRoutine blueProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueProcess3PieceRoutine");

  AutoTrajectory blueCenterToL4F = blueProcess3PieceRoutine.trajectory("blueCenterToL4F");
  AutoTrajectory blueL4FToSource = blueProcess3PieceRoutine.trajectory("blueL4FToSource");
  AutoTrajectory blueSourceToL4D = blueProcess3PieceRoutine.trajectory("blueSourceToL4D");
  AutoTrajectory blueL4DToSource = blueProcess3PieceRoutine.trajectory("blueL4DToSource");
  AutoTrajectory blueSourceToL4C = blueProcess3PieceRoutine.trajectory("blueSourceToL4C");

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4F.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // Score at L4F
    blueProcess3PieceRoutine.active().onTrue(blueCenterToL4F.cmd());
    scoreToL4Then(blueCenterToL4F, 1.0, blueL4FToSource.cmd());

    // Grab some coral then score at L4D
    grabSomeCoralThen(blueL4FToSource, blueSourceToL4D.cmd());
    scoreToL4Then(blueSourceToL4D, 1.3, blueL4DToSource.cmd());

    // Grab some coral then score at L4C
    grabSomeCoralThen(blueL4DToSource, blueSourceToL4C.cmd());
    scoreToL4Then(blueSourceToL4C, 1.3, elevator.coralL1PositionCG());

    return blueProcess3PieceRoutine;
  }
}
