package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class LEDs extends SubsystemBase {

  private final AddressableLED m_led = new AddressableLED(LedConstants.kLedPort);
  private final AddressableLEDBuffer m_ledData =
      new AddressableLEDBuffer(LedConstants.kLedBufferLength);

  public LEDs() {
    m_led.setLength(m_ledData.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledData);
  }

  public Command createEnabledCommand() {
    return this.run(
        () -> {
          // TODO: Display visual feedback about the state of the robot in enabled modes
        });
  }

  public Command createDisabledCommand(
      IntSupplier autoSwitchPositionSupplier,
      Supplier<Alliance> allianceColorSupplier,
      BooleanSupplier agreementInAllianceInputs) {
    return this.run(
        () -> {
          // TODO: Display visual feedback about the state of the robot in disabled mode
          allianceColorSupplier.get();
          autoSwitchPositionSupplier.getAsInt();
          agreementInAllianceInputs.getAsBoolean();
        });
  }

  public Command createChangeAutoAnimationCommand() {
    return this.run(
        () -> {
          // TODO: Play an animation when the auto selection changes
        });
  }
}
