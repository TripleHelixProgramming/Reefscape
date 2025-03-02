package frc.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class Util {
  public static Color allianceToColor(Alliance alliance) {
    return alliance == Alliance.Blue ? Color.kBlue : Color.kRed;
  }
}
