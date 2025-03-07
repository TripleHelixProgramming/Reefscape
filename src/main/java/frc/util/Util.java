package frc.util;

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
    return Math.abs(a - b) < 1e-6;
  }
}
