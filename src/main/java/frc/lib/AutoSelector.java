package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.ChoreoAuto;
import java.util.List;
import java.util.function.Supplier;

public class AutoSelector {

  private ChoreoAuto m_currentAuto;
  private DigitalInput[] m_switchPositions;
  private Supplier<Alliance> m_allianceColorSupplier;
  private List<AutoOption> m_autoOptions;
  private EventLoop m_loop = new EventLoop();

  public BooleanEvent m_changedAuto;

  /**
   * Constructs an autonomous selector switch
   *
   * @param ports An array of DIO ports for selecting an autonomous mode
   * @param allianceColorSupplier A method that supplies the current alliance color
   * @param autoOptions An array of autonomous mode options
   */
  public AutoSelector(
      int[] ports, Supplier<Alliance> allianceColorSupplier, List<AutoOption> autoOptions) {
    this.m_allianceColorSupplier = allianceColorSupplier;
    this.m_autoOptions = autoOptions;

    m_switchPositions = new DigitalInput[ports.length];
    m_changedAuto = new BooleanEvent(m_loop, () -> updateAuto());

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
    Alliance color = m_allianceColorSupplier.get();

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
    if (m_newAuto.equals(m_currentAuto)) return false;
    else {
      m_currentAuto = m_newAuto;
      return true;
    }
  }

  /** Schedules the command corresponding to the selected autonomous mode */
  public void scheduleAuto() {
    if (m_currentAuto != null) m_currentAuto.schedule();
  }

  /** Deschedules the command corresponding to the selected autonomous mode */
  public void cancelAuto() {
    if (m_currentAuto != null) m_currentAuto.cancel();
  }

  public void disabledPeriodic() {
    m_loop.poll();

    if (m_currentAuto != null) {
      SmartDashboard.putString("Auto", m_currentAuto.getName());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }
  }
}
