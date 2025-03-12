package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class Util {
  /**
   * Map alliance object to corresponding color.
   *
   * @param alliance the alliance whose color we want
   * @return the appropriate color
   */
  public static Color allianceToColor(Alliance alliance) {
    return alliance == Alliance.Blue ? Color.kBlue : Color.kRed;
  }

  public static boolean nearlyEqual(double a, double b) {
    return Math.abs(a - b) < Math.ulp(1);
  }

  /**
   * Get the reef sector in which the specified pose falls, for the
   * reef with the given center.  The reef is divided into 6 sectors,
   * each 60 degrees wide.  The sectors are numbered 0 to 5, starting
   * from the edge facing the driver station, and proceding clockwise.
   * So sector 0 has the range (-30, 30] (relative to driver station).
   * 
   * @param reefCenter the center of the reef
   * @param atPose the pose for which we want to know the reef sector
   * @return the index of the reef sector
   */
  public static int getReefSector(Pose2d reefCenter, Pose2d atPose) {
    var delta = atPose.getTranslation().minus(reefCenter.getTranslation());
    int vectorAngle = (int)delta.getAngle().getDegrees();
    int rayAngle = (vectorAngle + 30) % 360;
    return rayAngle / 60;
  }
}
