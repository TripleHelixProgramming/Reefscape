package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants.AllianceColor;

import java.util.function.BooleanSupplier;

public class AllianceSelector extends SubsystemBase{

  private DigitalInput m_allianceSelectionSwitch;
  
  public AllianceSelector(int port) {
    this.m_allianceSelectionSwitch = new DigitalInput(port);
  }

  public BooleanSupplier fieldRotatedSupplier() {
    return () -> m_allianceSelectionSwitch.get();
  }

  public AllianceColor getAllianceColor() {
    return m_allianceSelectionSwitch.get() ? AllianceColor.Red : AllianceColor.Blue;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Alliance Color", getAllianceColor().name());
  }
}
