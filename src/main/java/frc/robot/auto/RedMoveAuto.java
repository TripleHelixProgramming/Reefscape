package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import java.util.Optional;

public class RedMoveAuto extends AutoMode {

  public RedMoveAuto(Drivetrain drivetrain) {
    super(drivetrain);
  }

  AutoRoutine redMoveAutoRoutine = super.getAutoFactory().newRoutine("RedMoveRoutine");

  AutoTrajectory redMoveTrajectory = redMoveAutoRoutine.trajectory("redMoveAuto");

  @Override
  public String getName() {
    return "RedMoveAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redMoveTrajectory.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    redMoveAutoRoutine.active().onTrue(redMoveTrajectory.cmd());
    // spotless:on

    return redMoveAutoRoutine;
  }
}
