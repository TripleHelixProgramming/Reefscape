package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants.AllianceColor;

public class AllianceSelector {

  private AllianceColor m_currentColor;
  private DigitalInput m_allianceSelectionSwitch;
  private EventLoop m_loop;

  public AllianceSelector(int port) {
    this.m_allianceSelectionSwitch = new DigitalInput(port);
  }

  private boolean updateAlliance() {
    AllianceColor m_newColor =
        m_allianceSelectionSwitch.get() ? AllianceColor.Red : AllianceColor.Blue;
    if (m_newColor == m_currentColor) return false;
    else {
      m_currentColor = m_newColor;
      return true;
    }
  }

  public BooleanEvent changedAlliance = new BooleanEvent(m_loop, () -> updateAlliance());

  public boolean fieldRotated() {
    return m_currentColor.equals(AllianceColor.Red);
  }

  public AllianceColor getAllianceColor() {
    return m_currentColor;
  }

  public void disabledPeriodic() {
    m_loop.poll();
    SmartDashboard.putString("Alliance Color", getAllianceColor().name());
  }
}
