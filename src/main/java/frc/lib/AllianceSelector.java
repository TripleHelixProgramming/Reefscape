package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AllianceSelector {

  private Alliance m_currentColor;
  private DigitalInput m_allianceSelectionSwitch;
  private EventLoop m_loop = new EventLoop();

  public BooleanEvent m_changedAlliance;
  public BooleanEvent m_agreementInAllianceInputs;

  /**
   * Constructs an alliance color selector switch
   *
   * @param port DIO port for reading the alliance color input
   */
  public AllianceSelector(int port) {
    this.m_allianceSelectionSwitch = new DigitalInput(port);

    m_changedAlliance = new BooleanEvent(m_loop, () -> updateAlliance());
    m_agreementInAllianceInputs = new BooleanEvent(m_loop, () -> agreementInAllianceInputs());
  }

  private Alliance getAllianceFromSwitch() {
    return m_allianceSelectionSwitch.get() ? Alliance.Red : Alliance.Blue;
  }

  private boolean updateAlliance() {
    Alliance m_newColor = getAllianceFromSwitch();

    if (m_newColor.equals(m_currentColor)) return false;
    else {
      m_currentColor = m_newColor;
      return true;
    }
  }

  private boolean agreementInAllianceInputs() {
    Optional<Alliance> allianceFromFMS = DriverStation.getAlliance();
    Alliance allianceFromSwitch = getAllianceFromSwitch();

    if (allianceFromFMS.isPresent()) {
      return allianceFromSwitch.equals(allianceFromFMS.get());
    } else return false;
  }

  /**
   * @return Whether the field is rotated from the driver's perspective
   */
  public boolean fieldRotated() {
    return m_currentColor.equals(Alliance.Red);
  }

  /**
   * @return The current alliance
   */
  public Alliance getAllianceColor() {
    return m_currentColor;
  }

  public void disabledPeriodic() {
    m_loop.poll();
    SmartDashboard.putString("Alliance Color", getAllianceColor().name());
  }
}
