package frc.game;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

/**
 * Enumerates the Reefs on the field.
 *
 * <p>Provides methods for finding the nearest reef to a pose, and for finding the nearest reef face
 * to a pose.
 */
public enum Reef {
  Blue(new Pose2d(Inches.of(176.75), Inches.of(158.5), new Rotation2d())),
  Red(new Pose2d(Inches.of(514.125), Inches.of(158.5), new Rotation2d(Math.PI)));

  public static final double radius = Inches.of(50.25).in(Meters);
  public static final Pose2d blueCenter =
      new Pose2d(Inches.of(176.75), Inches.of(158.5), new Rotation2d());
  public static final Pose2d redCenter =
      new Pose2d(Inches.of(514.125), Inches.of(158.5), new Rotation2d(Math.PI));
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
    var deltaRed = atPose.getTranslation().minus(redCenter.getTranslation()).getNorm();
    var deltaBlue = atPose.getTranslation().minus(blueCenter.getTranslation()).getNorm();
    return deltaRed < deltaBlue ? Red : Blue;
  }

  private Pose2d centerPose;

  Reef(Pose2d centerPose) {
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
   * Get the reef sector in which the specified pose falls, for the reef with the given center. The
   * reef is divided into 6 sectors, each 60 degrees wide. The sectors are numbered 0 to 5, starting
   * from the edge facing the driver station, and proceding clockwise. So sector 0 has the range
   * (-30, 30] (relative to driver station).
   *
   * @param reefCenter the center of the reef
   * @param atPose the pose for which we want to know the reef sector
   * @return the index of the reef sector
   */
  public int getSector(Pose2d atPose) {
    var delta = atPose.getTranslation().minus(centerPose.getTranslation());
    int vectorAngle = (int) delta.getAngle().getDegrees();
    int rayAngle = (vectorAngle + 30) % 360;
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
    return Face.values()[sector + ordinal() * 6];
  }

  /**
   * Enumerates all of the Reef faces on the field. Each face is named for its alliance color and
   * the two letters corresponding to its coral pipes as desccribed in GM 5.3
   *
   * <p>Methods are provided to get poses along the face corresponding to the center, left coral
   * pipe, and right coral pipe.
   */
  public enum Face {
    blueAB(blueCenter, 0),
    blueCD(blueCenter, 1),
    blueEF(blueCenter, 2),
    blueGH(blueCenter, 3),
    blueIJ(blueCenter, 4),
    blueKL(blueCenter, 5),
    redAB(redCenter, 0),
    redCD(redCenter, 1),
    redEF(redCenter, 2),
    redGH(redCenter, 3),
    redIJ(redCenter, 4),
    redKL(redCenter, 5);

    private Pose2d centerPose;
    private Pose2d leftPipePose;
    private Pose2d rightPipePose;

    Face(Pose2d reefCenter, int index) {
      var rotation = new Rotation2d(Degrees.of(60.0)).times(index);
      var translation = new Translation2d(radius, rotation.plus(new Rotation2d(Math.PI)));
      var offset = new Transform2d(translation, rotation);
      centerPose = reefCenter.plus(offset);
      leftPipePose =
          centerPose.transformBy(
              new Transform2d(new Translation2d(0, -pipeSpacing.in(Inches) / 2), Rotation2d.kZero));
      rightPipePose =
          centerPose.transformBy(
              new Transform2d(new Translation2d(0, +pipeSpacing.in(Inches) / 2), Rotation2d.kZero));
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
     * Gets the left pipe pose of this face.
     *
     * @return the left pipe pose of this face
     */
    public Pose2d getLeftPipePose() {
      return leftPipePose;
    }

    /**
     * Gets the right pipe pose of this face.
     *
     * @return the right pipe pose of this face
     */
    public Pose2d getRightPipePose() {
      return rightPipePose;
    }
  }
}
