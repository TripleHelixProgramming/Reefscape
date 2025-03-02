package frc.robot.LEDs;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.util.Gamepiece;
import frc.util.Util;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class LEDs extends SubsystemBase {

  private static LEDs instance;
  private static final AddressableLED mainLed = new AddressableLED(LedConstants.kLedPort);
  private static final AddressableLEDBuffer mainLedBuffer =
      new AddressableLEDBuffer(LedConstants.kLedPixelCount);
  private static final AddressableLEDBufferView leftStrip = mainLedBuffer.createView(0, 19);
  private static final AddressableLEDBufferView rightStrip = mainLedBuffer.createView(20, 39);

  enum Segments implements LEDReader, LEDWriter {
    rightBottom(mainLed, mainLedBuffer, 0, 8, false),
    rightMiddle(mainLed, mainLedBuffer, 9, 10, false),
    rightTop(mainLed, mainLedBuffer, 11, 19, false),
    leftTop(mainLed, mainLedBuffer, 20, 28, true),
    leftMiddle(mainLed, mainLedBuffer, 29, 30, true),
    leftBottom(mainLed, mainLedBuffer, 31, 39, true);

    public static final Segments[] ALL = values();
    public static final Segments[] LEFT = {leftTop, leftMiddle, leftBottom};
    public static final Segments[] RIGHT = {rightTop, rightMiddle, rightBottom};
    public static final Segments[] TOP = {leftTop, rightTop};
    public static final Segments[] MIDDLE = {leftMiddle, rightMiddle};
    public static final Segments[] BOTTOM = {leftBottom, rightBottom};

    public final AddressableLED led;
    public final AddressableLEDBuffer buffer;
    public final AddressableLEDBufferView bufferView;
    public final int start;
    public final int end;

    Segments(
        AddressableLED led, AddressableLEDBuffer ledBuffer, int start, int end, boolean reversed) {
      this.led = led;
      this.buffer = ledBuffer;
      this.start = start;
      this.end = end;
      if (reversed) {
        this.bufferView = ledBuffer.createView(start, end).reversed();
      } else {
        this.bufferView = ledBuffer.createView(start, end);
      }
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
      buffer.setRGB(index, r, g, b);
    }

    @Override
    public int getLength() {
      return buffer.getLength();
    }

    @Override
    public int getRed(int index) {
      return buffer.getRed(index);
    }

    @Override
    public int getGreen(int index) {
      return buffer.getGreen(index);
    }

    @Override
    public int getBlue(int index) {
      return buffer.getBlue(index);
    }
  }

  public static synchronized LEDs getInstance() {
    if (instance == null) {
      mainLed.setLength(mainLedBuffer.getLength());
      mainLed.start();
      instance = new LEDs();
    }
    return instance;
  }

  private LEDs() {}

  public void update() {
    mainLed.setData(mainLedBuffer);
  }

  @Override
  public void periodic() {
    update();
  }

  public void setPixel(int index, Color color, LEDWriter... writers) {
    Arrays.stream(writers).forEach(writer -> writer.setLED(index, color));
  }

  public void setPixels(int start, int len, Color color, LEDWriter... writers) {
    while (len-- > 0) {
      setPixel(start++, color, writers);
    }
  }

  public void applyPattern(LEDPattern pattern, AddressableLEDBuffer... buffers) {
    Arrays.stream(buffers).forEach(buffer -> pattern.applyTo(buffer));
  }

  public void applyPattern(LEDPattern pattern, AddressableLEDBufferView... bufferViews) {
    Arrays.stream(bufferViews).forEach(bufferView -> pattern.applyTo(bufferView));
  }

  public void applyPattern(LEDPattern pattern, Segments... segments) {
    Arrays.stream(segments).forEach(segment -> pattern.applyTo(segment.bufferView));
  }

  public void fill(Color color, Segments... segments) {
    applyPattern(LEDPattern.solid(color), segments);
  }

  public void clear(Segments... segments) {
    fill(Color.kBlack, segments);
  }

  public void stackBlocks(
      Color color, int numBlocks, int blockSize, int blockSpace, LEDWriter... writers) {
    for (int i = 0; i < numBlocks * (blockSize + blockSpace); i += blockSize + blockSpace) {
      setPixels(i, blockSize, color, writers);
      setPixels(i + blockSize, blockSpace, Color.kBlack, writers);
    }
  }

  public void displayAutoSelection(Alliance alliance, int autoMode, boolean agreement) {
    clear(Segments.ALL);
    if (autoMode > 0) {
      stackBlocks(
          Util.allianceToColor(alliance),
          autoMode,
          LedConstants.kLEDsPerBlock,
          LedConstants.kLEDsBetweenBlocks,
          leftStrip,
          rightStrip);
    } else { // No auto selected.
      setPixels(0, LedConstants.kLEDsPerBlock, Color.kYellow, leftStrip, rightStrip);
    }
    /*
     * If there's disagreement about the alliance inputs, light two yellow pixels
     * at the top of either strip.
     */
    if (!agreement) {
      setPixels(0, 2, Color.kYellow, leftStrip.reversed(), rightStrip.reversed());
    }
    update();
  }

  public void displayDefaultInfo(boolean isAlgaeMode, Optional<Gamepiece> gamepiece) {
    // Middle LEDs indicate control mode
    fill(isAlgaeMode ? LedConstants.algaeColor : LedConstants.coralColor, Segments.MIDDLE);
    // Top and Bottom LEDs indicate gamepiece possession
    var pieceColor = gamepiece.isPresent() ? gamepiece.get().color : Color.kBlack;
    fill(pieceColor, Segments.TOP);
    fill(pieceColor, Segments.BOTTOM);
    update();
  }

  /*
   * Use LEDs to indicate how to move the robot relative to its current
   * position to reach the target pose.  The directions must indicate:
   *
   * (1) Heading - how to face the robot in the correct direction
   *   - Use middle segments: Both white indicates proper heading.
   *     Both blue means rotate clockwise, both magenta means counter.
   *
   * (2) X
   *   - Use top segments: Green means move forward, red means backwards.
   *
   * (3) Y
   *   - Use bottom segments: Illuminate the corresponding side.
   *
   * (4) Magnitude of change:
   *   - Use gradients for colors: brighter means more change.
   */
  public void displayPoseSeek(Pose2d currentPose, Pose2d targetPose) {
    var delta = targetPose.minus(currentPose);
  }

  public Command createEnabledCommand(BooleanSupplier isAlgeMode, Supplier<Gamepiece> gamepiece) {
    return this.run(
        () -> {
          displayDefaultInfo(isAlgeMode.getAsBoolean(), Optional.ofNullable(gamepiece.get()));
        });
  }

  public Command createPoseSeekingCommand(
      Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> currentPoseSupplier) {
    return this.run(() -> displayPoseSeek(currentPoseSupplier.get(), targetPoseSupplier.get()))
        .ignoringDisable(true);
  }

  public Command createDisabledCommand(
      IntSupplier autoSwitchPositionSupplier,
      Supplier<Alliance> allianceColorSupplier,
      BooleanSupplier agreementInAllianceInputs) {
    return this.run(
            () ->
                displayAutoSelection(
                    allianceColorSupplier.get(),
                    autoSwitchPositionSupplier.getAsInt(),
                    agreementInAllianceInputs.getAsBoolean()))
        .ignoringDisable(true);
  }

  public Command createChangeAutoAnimationCommand() {
    Time duration = Seconds.of(0.5);
    LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    LEDPattern scrollingRainbow = rainbow.scrollAtRelativeSpeed(duration.asFrequency());
    return this.run(() -> applyPattern(scrollingRainbow, leftStrip, rightStrip))
        .withTimeout(duration)
        .ignoringDisable(true);
  }

  public static AddressableLEDBufferView[] intakeBuffers = {
    Segments.leftTop.bufferView.reversed(),
    Segments.rightTop.bufferView.reversed(),
    Segments.leftBottom.bufferView,
    Segments.rightBottom.bufferView
  };

  public static AddressableLEDBufferView[] outtakeBuffers = {
    Segments.leftTop.bufferView,
    Segments.rightTop.bufferView,
    Segments.leftBottom.bufferView.reversed(),
    Segments.rightBottom.bufferView.reversed()
  };

  public Command createRollerAnimationCommand(boolean isIntake) {
    Time duration = Seconds.of(0.5);
    LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    LEDPattern scrollingRainbow = rainbow.scrollAtRelativeSpeed(duration.asFrequency());

    return this.run(() -> applyPattern(scrollingRainbow, isIntake ? intakeBuffers : outtakeBuffers))
        .withTimeout(duration)
        .ignoringDisable(true);
  }
}
