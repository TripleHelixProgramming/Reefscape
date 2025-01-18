package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoSelector {

  private Optional<Command> m_currentAuto;
  private DigitalInput[] m_switchPositions;
  private Supplier<Alliance> m_allianceColorSupplier;
  private List<AutoOption> m_autoOptions;
  private EventLoop m_loop = new EventLoop();
  private BooleanEvent m_changedAutoSelection;

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
    for (int i = 0; i < ports.length; i++) {
      m_switchPositions[i] = new DigitalInput(ports[i]);
    }

    m_changedAutoSelection = new BooleanEvent(m_loop, () -> updateAuto());
  }

  /**
   * @return The position of the autonomous selection switch
   */
  public int getSwitchPosition() {
    for (int i = 0; i < m_switchPositions.length; i++) {
      if (!m_switchPositions[i].get()) {
        return i + 1;
      }
    }
    return 0; // failure of the physical switch
  }

  private Alliance getAllianceColor() {
    return m_allianceColorSupplier.get();
  }

  private Optional<Command> findMatchingOption() {
    return m_autoOptions.stream()
        .filter(o -> o.getColor() == getAllianceColor())
        .filter(o -> o.getOption() == getSwitchPosition())
        .findFirst()
        .flatMap(AutoOption::getAutoCommand);
  }

  private boolean updateAuto() {
    Optional<Command> m_newAuto = findMatchingOption();
    if (m_newAuto.equals(m_currentAuto)) return false;
    else {
      m_currentAuto = m_newAuto;
      return true;
    }
  }

  /**
   * @return Object for binding a command to a change in autonomous mode selection
   */
  public Trigger getChangedAutoSelection() {
    return m_changedAutoSelection.castTo(Trigger::new);
  }

  /** Schedules the command corresponding to the selected autonomous mode */
  public void scheduleAuto() {
    m_currentAuto.ifPresent(o -> o.schedule()); 
  }

  /** Deschedules the command corresponding to the selected autonomous mode */
  public void cancelAuto() {
    m_currentAuto.ifPresent(o -> o.cancel());
  }

  public void disabledPeriodic() {
    m_loop.poll();

    SmartDashboard.putNumber("Auto Selector Switch Position", getSwitchPosition());

    if (m_currentAuto.isPresent()) {
      SmartDashboard.putString("Auto", m_currentAuto.get().getName());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }
  }
}
