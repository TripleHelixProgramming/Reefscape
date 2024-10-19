package frc.lib;

import frc.robot.Constants.AutoConstants.AllianceColor;
import frc.robot.autos.ChoreoAuto;

public class AutoOption {
  private AllianceColor m_color;
  private int m_option;
  private ChoreoAuto m_auto;

  public AutoOption(AllianceColor color, int option, ChoreoAuto auto) {
    this.m_color = color;
    this.m_option = option;
    this.m_auto = auto;
  }

  public AutoOption(AllianceColor color, int option) {
    this.m_color = color;
    this.m_option = option;
    this.m_auto = null;
  }

  public AllianceColor getColor() {
    return this.m_color;
  }

  public int getOption() {
    return this.m_option;
  }

  public ChoreoAuto getChoreoAuto() {
    return this.m_auto;
  }
}
