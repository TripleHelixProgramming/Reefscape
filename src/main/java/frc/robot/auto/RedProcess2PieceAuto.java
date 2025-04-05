package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class RedProcess2PieceAuto extends L4MultiAuto {

  public RedProcess2PieceAuto(Drivetrain drivetrain, Elevator elevatorSystem) {
    super("RedProcess2PieceAuto", drivetrain, elevatorSystem);
  }

  AutoRoutine redProcess3PieceRoutine =
      super.getAutoFactory().newRoutine("redProcess2PieceRoutine");

  AutoTrajectory redCenterToL4F = redProcess3PieceRoutine.trajectory("redCenterToL4F");
  AutoTrajectory redL4FToSource = redProcess3PieceRoutine.trajectory("redL4FToSource");
  AutoTrajectory redSourceToL4D = redProcess3PieceRoutine.trajectory("redSourceToL4D");

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redCenterToL4F.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // Score at L4F
    redProcess3PieceRoutine.active().onTrue(redCenterToL4F.cmd());
    scoreToL4Then(redCenterToL4F, 1.0, redL4FToSource.cmd());

    // Grab some coral then score at L4D
    grabSomeCoralThen(redL4FToSource, redSourceToL4D.cmd());
    scoreToL4Then(redSourceToL4D, 2.0, coralRoller.stop());

    return redProcess3PieceRoutine;
  }
}
