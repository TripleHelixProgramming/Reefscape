package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants.AllianceColor;
import frc.robot.autos.ChoreoAuto;
import java.util.List;

public class AutoSelector {

  private ChoreoAuto m_currentAuto;
  private DigitalInput[] m_switchPositions;
  private AllianceSelector m_allianceSelector;
  private List<AutoOption> m_autoOptions;
  private EventLoop m_loop;

  public AutoSelector(
      int[] ports, AllianceSelector allianceSelector, List<AutoOption> autoOptions) {
    this.m_allianceSelector = allianceSelector;
    this.m_autoOptions = autoOptions;

    m_switchPositions = new DigitalInput[ports.length];

    for (int i = 0; i < ports.length; i++) {
      m_switchPositions[i] = new DigitalInput(ports[i]);
    }
  }

  private int getSwitchPosition() {
    for (int i = 0; i < m_switchPositions.length; i++) {
      if (!m_switchPositions[i].get()) {
        return i + 1;
      }
    }
    return 0; // failure of the physical switch
  }

  private ChoreoAuto findMatchingOption() {
    int switchPosition = getSwitchPosition();
    AllianceColor color = m_allianceSelector.getAllianceColor();

    for (int i = 0; i < m_autoOptions.size(); i++) {
      if (m_autoOptions.get(i).getColor() == color)
        if (m_autoOptions.get(i).getOption() == switchPosition) {
          return m_autoOptions.get(i).getChoreoAuto();
        }
    }
    return null;
  }

  private boolean updateAuto() {
    ChoreoAuto m_newAuto = findMatchingOption();
    if (m_newAuto == m_currentAuto) return false;
    else {
      m_currentAuto = m_newAuto;
      return true;
    }
  }

  public BooleanEvent changedAuto = new BooleanEvent(m_loop, () -> updateAuto());

  public void scheduleAuto() {
    if (m_currentAuto != null) m_currentAuto.schedule();
  }

  public void cancelAuto() {
    if (m_currentAuto != null) m_currentAuto.cancel();
  }

  public void disabledPeriodic() {
    m_loop.poll();

    m_allianceSelector.changedAlliance.ifHigh(() -> updateAuto());

    if (m_currentAuto != null) {
      SmartDashboard.putString("Auto", m_currentAuto.getName());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }
  }
}
