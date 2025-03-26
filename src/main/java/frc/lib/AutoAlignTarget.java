package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

public abstract class AutoAlignTarget {
  private Pose2d newPose;

  /** Return the pose associated with this target */
  public abstract Pose2d getPose();

  /** Set the pose associated with this target */
  public void setPose(Pose2d pose) {
    newPose = pose;
  }

  public Optional<Pose2d> getNewPose() {
    return Optional.ofNullable(newPose);
  }

  /** Store the set pose as an override for future uses of the target pose. */
  public abstract void memoize();
}
