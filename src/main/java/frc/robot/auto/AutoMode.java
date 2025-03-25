package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.drivetrain.Drivetrain;
import java.util.Optional;

public abstract class AutoMode {
  private final AutoFactory autoFactory;

  public AutoMode(Drivetrain drivetrain) {
    autoFactory =
        new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::followTrajectory,
            false,
            drivetrain);
  }

  public AutoFactory getAutoFactory() {
    return this.autoFactory;
  }

  public abstract AutoRoutine getAutoRoutine();

  public abstract Optional<Pose2d> getInitialPose();

  public abstract String getName();
}
