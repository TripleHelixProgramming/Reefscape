package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;

public interface AutoAlignTarget {

  /** Return the pose associated with this target */
  public Pose2d getPose();

  /** Store the supplied pose as an override for future uses of the target pose. */
  public void memoize(Pose2d pose);
}
