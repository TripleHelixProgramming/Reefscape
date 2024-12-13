package frc.robot.LEDs;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
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

  private void clearBuffer() {
    for (var i = 0; i < m_ledData.getLength(); i++) {
      m_ledData.setRGB(i, 0, 0, 0);
    }
  }

  private void autoColor(Alliance alliance, int autoMode) {
    clearBuffer();
    int block = LedConstants.kLEDsPerBlock + LedConstants.kLEDsBetweenBlocks;
    if (1 > autoMode) { // 0 indicates no auto selected.
      for (var led = 0; led < LedConstants.kLEDsPerBlock; led++) {
        m_ledData.setLED(led, Color.kYellow);
      }
    } else {
      for (var mode = 0; mode < autoMode; mode++) {
        for (var i = 0; i < LedConstants.kLEDsPerBlock; i++) {
          if (alliance.equals(Alliance.Red)) {
            m_ledData.setLED(i + (mode * block), Color.kRed);
          } else {
            m_ledData.setLED(i + (mode * block), Color.kBlue);
          }
        }
      }
    }
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
              // TODO: Change the basic disabled pattern (e.g. blink or pulse the LED strip) if the
              // selected alliance color does not match the value from FMS
              autoColor(allianceColorSupplier.get(), autoSwitchPositionSupplier.getAsInt());
            })
        .ignoringDisable(true);
  }

  public Command createChangeAutoAnimationCommand() {
    Time duration = Seconds.of(0.5);
    LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    LEDPattern scrollingRainbow = rainbow.scrollAtRelativeSpeed(duration.asFrequency());
    return this.run(
            () -> {
              scrollingRainbow.applyTo(m_ledData);
            })
        .withTimeout(duration)
        .ignoringDisable(true);
  }
}
