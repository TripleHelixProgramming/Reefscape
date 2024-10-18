package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.AllianceColor;
import frc.robot.Robot.AutoOption;
import frc.robot.autos.ChoreoAuto;

import java.util.List;

public class AutoSelector {

  private DigitalInput[] m_switchPositions;
  private AllianceSelector m_allianceSelector;
  private List<AutoOption> m_autoOptions;

  private ChoreoAuto m_autonomous;

  public AutoSelector(int[] ports, AllianceSelector allianceSelector, List<AutoOption> autoOptions) {

    m_switchPositions = new DigitalInput[ports.length];
    for (int i = 0; i < ports.length; i++) {
      m_switchPositions[i] = new DigitalInput(ports[i]);
    }

    this.m_allianceSelector = allianceSelector;
    this.m_autoOptions = autoOptions;
  }

  /**
   * @return Index in array of Digital Inputs corresponding to selected auto mode
   */
  private int getSwitchPosition() {
    for (int i = 0; i < m_switchPositions.length; i++) {
      if (!m_switchPositions[i].get()) {
        return i + 1;
      }
    }
    return 0; // failure of the physical switch
  }
  
  public ChoreoAuto getSelectedAutonomous() {
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

  /**
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    getSelectedAutonomous();
    if (m_autonomous != null) return m_autonomous;
    else return null;
  }

  public void periodic() {

    if (m_autonomous != null) {
      SmartDashboard.putString("Auto", m_autonomous.getName());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }
  }
}
