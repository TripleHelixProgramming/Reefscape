package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import java.util.Optional;

public class BlueMoveAuto extends AutoMode {

  public BlueMoveAuto(Drivetrain drivetrain) {
    super(drivetrain);
  }

  AutoRoutine blueMoveAutoRoutine = super.getAutoFactory().newRoutine("BlueMoveRoutine");

  AutoTrajectory blueMoveTrajectory = blueMoveAutoRoutine.trajectory("blueMoveAuto");

  @Override
  public String getName() {
    return "BlueMoveAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueMoveTrajectory.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    blueMoveAutoRoutine.active().onTrue(blueMoveTrajectory.cmd());
    // spotless:on

    return blueMoveAutoRoutine;
  }
}
