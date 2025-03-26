package frc.game;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AutoAlignTarget;
import frc.lib.Config;
import java.util.Optional;

/**
 * Enumerates the Reefs on the field.
 *
 * <p>Provides methods for finding the nearest reef to a pose, and for finding the nearest reef face
 * to a pose.
 */
public enum Reef {
  Blue(new Pose2d(Inches.of(176.75), Inches.of(158.5), Rotation2d.kZero)),
  Red(new Pose2d(Inches.of(514.125), Inches.of(158.5), Rotation2d.kPi));

  /** The radius of the reef hexagon */
  public static final Distance radius = Inches.of(50.25);

  /** The spacing between the left and right pipe centers. */
  public static final Distance pipeSpacing = Inches.of(13); // GM 5.3

  /**
   * Gets the nearest reef to the given pose.
   *
   * <p>The nearest reef is determined by the distance between the given pose and the center of each
   * reef.
   *
   * @param atPose the pose for which to find the nearest reef
   * @return the nearest reef
   */
  public static Reef getNearestReef(Pose2d atPose) {
    var deltaRed = atPose.getTranslation().minus(Red.getCenterPose().getTranslation()).getNorm();
    var deltaBlue = atPose.getTranslation().minus(Blue.getCenterPose().getTranslation()).getNorm();
    return deltaRed < deltaBlue ? Red : Blue;
  }

  private Pose2d centerPose;

  Reef(Pose2d centerPose) {
    var adjust = Config.getDefault().loadTransform("Reef." + this.name());
    if (adjust.isPresent()) {
      centerPose = centerPose.transformBy(adjust.get());
    }
    this.centerPose = centerPose;
  }

  /**
   * Get the center of this reef.
   *
   * @return the pose for the center of this reef
   */
  public Pose2d getCenterPose() {
    return centerPose;
  }

  /**
   * Get the sector of this reef into which the specified pose falls. The reefs are divided into 6
   * sectors, each 60 degrees wide. The sectors are numbered 0 to 5, starting from the edge facing
   * the driver station, and proceding clockwise. So sector 0 has the range (-30, 30], relative to
   * driver station.
   *
   * <p>Note: this code works the same for Red as Blue alliance because the Red reef center pose has
   * a rotational component that sets the frame of reference for the pose subtraction. This is why a
   * pose in sector 1 of the Red reef does not appear to be in sector 4.
   *
   * @param atPose the pose for which we want to know the reef sector
   * @return the index of the reef sector
   */
  public int getSector(Pose2d atPose) {
    var delta = centerPose.getTranslation().minus(atPose.getTranslation());
    int vectorAngle = (int) delta.getAngle().getDegrees();
    int rayAngle = (vectorAngle + 30 + 360 + (int) centerPose.getRotation().getDegrees()) % 360;
    // SmartDashboard.putNumber("Sector delta X", delta.getX());
    // SmartDashboard.putNumber("Sector delta Y", delta.getY());
    // SmartDashboard.putNumber("Sector vector delta", vectorAngle);
    // SmartDashboard.putNumber("Sector ray angle", rayAngle);
    return rayAngle / 60;
  }

  /**
   * Get the face on this reef into whose sector the specified pose falls.
   *
   * @param atPose the pose for which we want to know the reef sector
   * @return the reef face
   */
  public Face getNearestFace(Pose2d atPose) {
    var sector = getSector(atPose);
    return Face.values()[6 * ordinal() + sector];
  }

  /**
   * Enumerates all of the Reef faces on the field. Each face is named for its alliance color and
   * the two letters corresponding to its coral pipes as desccribed in GM 5.3
   *
   * <p>Methods are provided to get poses along the face corresponding to the center, left coral
   * pipe, and right coral pipe.
   */
  public enum Face {
    blueAB(Blue, 0),
    blueCD(Blue, 1),
    blueEF(Blue, 2),
    blueGH(Blue, 3),
    blueIJ(Blue, 4),
    blueKL(Blue, 5),
    redAB(Red, 0),
    redCD(Red, 1),
    redEF(Red, 2),
    redGH(Red, 3),
    redIJ(Red, 4),
    redKL(Red, 5);

    private final Reef reef;
    private Pose2d centerPose;
    private Pose2d leftPipePose;
    private Pose2d rightPipePose;
    private Optional<Pose2d> memoLeftPipePose;
    private Optional<Pose2d> memoRightPipePose;

    class PipeTarget extends AutoAlignTarget {
      private final boolean isLeft;

      public PipeTarget(boolean isLeft) {
        this.isLeft = isLeft;
      }

      @Override
      public Pose2d getPose() {
        return isLeft ? getLeftPipePose() : getRightPipePose();
      }

      @Override
      public void memoize() {
        getNewPose().ifPresent(pose -> {
          if (isLeft) {
            memoizeLeftPipePose(pose);
          } else {
            memoizeRightPipePose(pose);
          }
        });
      }
    }

    Face(Reef reef, int index) {
      this.reef = reef;
      var rotation = new Rotation2d(Degrees.of(60.0)).times(index);
      var translation = new Translation2d(radius.in(Meters), rotation.plus(Rotation2d.kPi));
      var offset = new Transform2d(translation, rotation);
      centerPose = reef.getCenterPose().plus(offset);
      leftPipePose =
          centerPose.transformBy(
              new Transform2d(
                  new Translation2d(Meters.of(0), pipeSpacing.div(2)), Rotation2d.kZero));
      rightPipePose =
          centerPose.transformBy(
              new Transform2d(
                  new Translation2d(Meters.of(0), pipeSpacing.div(-2)), Rotation2d.kZero));

      memoLeftPipePose = Config.getDefault().loadPose2d("Reef.Face." + this.name() + ".leftPipe");
      memoRightPipePose = Config.getDefault().loadPose2d("Reef.Face." + this.name() + ".rightPipe");
      SmartDashboard.putString("Reef.Face." + this.name(), centerPose.toString());
    }

    /**
     * Gets the reef this face belongs to.
     *
     * @return the reef this face belongs to
     */
    public Reef getReef() {
      return reef;
    }

    /**
     * Gets the center pose of this face.
     *
     * @return the center pose of this face
     */
    public Pose2d getCenterPose() {
      return centerPose;
    }

    /**
     * Return the left pipe as an auto-align target
     *
     * @return left pipe target
     */
    public AutoAlignTarget getLeftPipe() {
      return new PipeTarget(true);
    }

    /**
     * Return the right pipe as an auto-align target
     *
     * @return right pipe target
     */
    public AutoAlignTarget getRightPipe() {
      return new PipeTarget(false);
    }

    /**
     * Gets the left pipe pose of this face.
     *
     * @return the left pipe pose of this face
     */
    public Pose2d getLeftPipePose() {
      if (memoLeftPipePose.isEmpty() && memoRightPipePose.isPresent()) {
        var transform =
            new Transform2d(new Translation2d(Meters.of(0), pipeSpacing), Rotation2d.kZero);
        memoLeftPipePose = Optional.of(memoRightPipePose.get().transformBy(transform));
      }
      return memoLeftPipePose.orElse(leftPipePose);
    }

    /**
     * Gets the right pipe pose of this face.
     *
     * @return the right pipe pose of this face
     */
    public Pose2d getRightPipePose() {
      if (memoRightPipePose.isEmpty() && memoLeftPipePose.isPresent()) {
        var transform =
            new Transform2d(
                new Translation2d(Meters.of(0), pipeSpacing.unaryMinus()), Rotation2d.kZero);
        memoRightPipePose = Optional.of(memoLeftPipePose.get().transformBy(transform));
      }
      return memoRightPipePose.orElse(rightPipePose);
    }

    /**
     * Store the specified pose as an override for the derived value for the left pipe. Temporarily
     *
     * @param pose the override pose
     */
    public void memoizeLeftPipePose(Pose2d pose) {
      memoLeftPipePose = Optional.of(pose);
      Config.getDefault().storePose2d("Reef.Face." + this.toString() + ".leftPipe", pose);
    }

    /**
     * Store the specified pose as an override for the derived value for the left pipe.
     *
     * @param pose the override pose
     */
    public void memoizeRightPipePose(Pose2d pose) {
      memoRightPipePose = Optional.of(pose);
      Config.getDefault().storePose2d("Reef.Face." + this.toString() + ".rightPipe", pose);
    }
  }
}
