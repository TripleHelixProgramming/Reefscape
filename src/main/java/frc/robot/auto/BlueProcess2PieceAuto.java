package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class BlueProcess2PieceAuto extends L4MultiAuto {

  public BlueProcess2PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super("BlueProcess2PieceAuto", drivetrain, elevatorSystem);
  }

  AutoRoutine blueProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("blueProcess2PieceRoutine");

  AutoTrajectory blueCenterToL4F = blueProcess3PieceRoutine.trajectory("blueCenterToL4F");
  AutoTrajectory blueL4FToSource = blueProcess3PieceRoutine.trajectory("blueL4FToSource");
  AutoTrajectory blueSourceToL4D = blueProcess3PieceRoutine.trajectory("blueSourceToL4D");

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
    scoreToL4Then(blueSourceToL4D, 2.0, coralRoller.stop());

    return blueProcess3PieceRoutine;
  }
}
