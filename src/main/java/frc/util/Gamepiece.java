package frc.util;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LedConstants;

public enum Gamepiece {
  ALGAE(LedConstants.algaeColor),
  CORAL(LedConstants.coralColor);

  public final Color color;

  Gamepiece(Color color) {
    this.color = color;
  }
}
