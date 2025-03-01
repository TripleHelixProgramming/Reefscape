package frc.robot.LEDs;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LedConstants;

import java.time.Duration;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class LEDs extends SubsystemBase {

  private final AddressableLED led = new AddressableLED(LedConstants.kLedPort);
  private final AddressableLEDBuffer ledBuffer =
  private final AddressableLED led = new AddressableLED(LedConstants.kLedPort);
  private final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(LedConstants.kLedBufferLength);
  private final AddressableLEDBufferView leftStrip = ledBuffer.createView(20, 39).reversed();
  private final AddressableLEDBufferView leftStripTop = ledBuffer.createView(20, 28).reversed();
  private final AddressableLEDBufferView leftStripBottom = ledBuffer.createView(31, 39).reversed();
  private final AddressableLEDBufferView rightStrip = ledBuffer.createView(0, 19);
  private final AddressableLEDBufferView rightStripTop = ledBuffer.createView(11, 19);
  private final AddressableLEDBufferView rightStripBottom = ledBuffer.createView(0, 8);
  private final AddressableLEDBufferView leftStripMiddle = ledBuffer.createView(29, 30).reversed();
  private final AddressableLEDBufferView rightStripMiddle = ledBuffer.createView(9, 10);

  public LEDs() {
    led.setLength(ledBuffer.getLength());
    led.start();
    led.setLength(ledBuffer.getLength());
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
    led.setData(ledBuffer);
  }

  private void clearBuffer() {
   setAllLights(Color.kBlack);
  }

  private void setMiddle(Color color) {
    LEDPattern solid = LEDPattern.solid(color);
    solid.applyTo(leftStripMiddle);
    solid.applyTo(rightStripMiddle);
  }

  private void setBoth(int index, Color color) {
    leftStrip.setLED(index, color);
    rightStrip.setLED(index, color);
  }

  private void autoColor(Alliance alliance, int autoMode) {
    clearBuffer();
    int block = LedConstants.kLEDsPerBlock + LedConstants.kLEDsBetweenBlocks;
    if (1 > autoMode) { // 0 indicates no auto selected.
      for (var led = 0; led < LedConstants.kLEDsPerBlock; led++) {
        setBoth(led, Color.kYellow);
        setBoth(led, Color.kYellow);
      }
    } else {
      for (var mode = 0; mode < autoMode; mode++) {
        for (var i = 0; i < LedConstants.kLEDsPerBlock; i++) {
          setBoth(i + (mode * block), alliance == Alliance.Red ? Color.kRed : Color.kBlue);
          setBoth(i + (mode * block), alliance == Alliance.Red ? Color.kRed : Color.kBlue);
        }
      }
    }
    led.setData(ledBuffer);
    led.setData(ledBuffer);
  }

  public Command createEnabledCommand() {
    return this.run(
        () -> {
          // TODO: Display visual feedback about the state of the robot in enabled modes
        });
  }

  /*
   * Use LEDs to indicate how to move the robot from the current pose
   * to the target pose.  The directions must indicate:
   *
   * (1) Heading - how to face the robot in the correct direction
   *   - Use outer pixels to indicate heading: Both green indicates
   *     proper heading.  Red on one side means rotate away from that
   *     side.
   *
   * (2) Distance - how far from target
   *   - Some portion of interior pixels light up.  More pixels
   *     means more distance.  Red means backwards, relative to
   *     the robot; green means forward, yellow means sideways.
   *
   * (3) Direction - which angle to move the robot
   *   - Offset of pixel block indicates which angle: close to center
   *     means pretty much straight, close to edge means diagonal.
   *
   */
  public void indicatePoseSeek(Pose2d currentPose, Pose2d targetPose) {
    var delta = targetPose.minus(currentPose);
  }

  public void createDefaultCommand(Trigger algaeMode) {
    setDefaultCommand(this.run(() -> setMiddle(algaeMode.getAsBoolean()? Color.kGreen : Color.kPink)));
  }

  public Command createPoseSeekingCommand(
      Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> currentPoseSupplier) {
    return this.run(() -> indicatePoseSeek(currentPoseSupplier.get(), targetPoseSupplier.get()))
        .ignoringDisable(true);
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
              scrollingRainbow.applyTo(leftStrip);
              scrollingRainbow.applyTo(rightStrip);
            })
        .withTimeout(duration)
        .ignoringDisable(true);
  }

  public void setAllLights(Color color){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }

  public Command createSolidColorCommand(final Color color){
    return this.run( () -> { setAllLights(color); });
  }

  

  public void biScroll(Color color, boolean ifInward, Time duration){
    LEDPattern solid = LEDPattern.solid(color);
    var scroll = solid.scrollAtRelativeSpeed(duration.asFrequency());
    
    
  }
}
