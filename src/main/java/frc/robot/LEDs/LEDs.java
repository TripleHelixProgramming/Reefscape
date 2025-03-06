package frc.robot.LEDs;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.AutoOption;
import frc.robot.Constants.LedConstants;
import frc.util.Gamepiece;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A subsystem to control the LEDs on the robot. The LEDs are controlled by an {@link
 * AddressableLED} object, which is used to set the color of each LED in the strip. The LEDs are
 * divided into segments, each of which can be controlled independently. The segments are defined in
 * the {@link Segments} enum, which implements the {@link LEDReader} and {@link LEDWriter}
 * interfaces.
 *
 * <p>The LEDs can be controlled by setting the color of individual pixels, or by applying a pattern
 * to a segment or group of segments. The {@link LEDs} class provides methods to set the color of
 * individual pixels, apply patterns to segments, and display information on the LEDs.
 *
 * <p>This is a singleton class. If more AddressableLED objects are needed, they can be added
 * alongside ther main LED object, and new segments can be defined in the {@link Segments} enum. In
 * this case, the update() method would need to be updated to include the new LED objects.
 */
public class LEDs extends SubsystemBase {

  private static LEDs instance;
  private static final AddressableLED mainLed = new AddressableLED(LedConstants.kLedPort);
  private static final AddressableLEDBuffer mainLedBuffer =
      new AddressableLEDBuffer(LedConstants.kLedPixelCount);
  private static final AddressableLEDBufferView leftStrip = mainLedBuffer.createView(0, 19);
  private static final AddressableLEDBufferView rightStrip =
      mainLedBuffer.createView(20, 39).reversed();

  /**
   * Enum to define the segments of the LED strip. Each segment is defined by an {@link
   * AddressableLED} object, an {@link AddressableLEDBuffer} object, a start index, an end index,
   * and a boolean indicating whether the segment is reversed. The enum implements the {@link
   * LEDReader} and {@link LEDWriter} interfaces, so LEDPattern objects can be applied to the
   * segments.
   */
  enum Segments implements LEDReader, LEDWriter {
    rightBottom(mainLed, mainLedBuffer, 0, 7, true),
    rightMiddle(mainLed, mainLedBuffer, 8, 11, true),
    rightTop(mainLed, mainLedBuffer, 12, 19, true),
    leftTop(mainLed, mainLedBuffer, 20, 27, false),
    leftMiddle(mainLed, mainLedBuffer, 28, 31, false),
    leftBottom(mainLed, mainLedBuffer, 32, 39, false);

    public static final Segments[] ALL = values();
    public static final Segments[] LEFT = {leftTop, leftMiddle, leftBottom};
    public static final Segments[] RIGHT = {rightTop, rightMiddle, rightBottom};
    public static final Segments[] TOP = {leftTop, rightTop};
    public static final Segments[] MIDDLE = {leftMiddle, rightMiddle};
    public static final Segments[] BOTTOM = {leftBottom, rightBottom};
    public static final Segments[] TOP_AND_BOTTOM = {leftTop, rightTop, leftBottom, rightBottom};

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

  /**
   * Get the singleton instance of the LEDs subsystem. If the instance does not exist, create it and
   * start the main LED strip.
   *
   * @return the LEDs instance
   */
  public static synchronized LEDs getInstance() {
    if (instance == null) {
      mainLed.setLength(mainLedBuffer.getLength());
      mainLed.start();
      instance = new LEDs();
    }
    return instance;
  }

  /** Private constructor to prevent instantiation of the LEDs subsystem. */
  private LEDs() {}

  /** Update the main LED strip with the current data in the main LED buffer. */
  public void update() {
    mainLed.setData(mainLedBuffer);
  }

  /**
   * Updates the LED with the current buffer.
   *
   * <p>{@inheritDoc}}
   */
  @Override
  public void periodic() {
    update();
  }

  /**
   * Set the color of a single pixel on the LED strip.
   *
   * @param index the index of the pixel to set
   * @param color the color to set the pixel to
   * @param writers the LEDWriters to write to
   */
  public void setPixel(int index, Color color, LEDWriter... writers) {
    Arrays.stream(writers).forEach(writer -> writer.setLED(index, color));
  }

  /**
   * Set the color of a range of pixels on the LED strip.
   *
   * @param start the start index of the range
   * @param len the length of the range
   * @param color the color to set the pixels to
   * @param writers the LEDWriters to write to
   */
  public void setPixels(int start, int len, Color color, LEDWriter... writers) {
    while (len-- > 0) {
      setPixel(start++, color, writers);
    }
  }

  /**
   * Apply a pattern to a group of LED buffers.
   *
   * @param pattern the pattern to apply
   * @param buffers the LED buffers to apply the pattern to
   */
  public void applyPattern(LEDPattern pattern, AddressableLEDBuffer... buffers) {
    Arrays.stream(buffers).forEach(buffer -> pattern.applyTo(buffer));
  }

  /**
   * Apply a pattern to a group of LED buffer views.
   *
   * @param pattern the pattern to apply
   * @param bufferViews the LED buffer views to apply the pattern to
   */
  public void applyPattern(LEDPattern pattern, AddressableLEDBufferView... bufferViews) {
    Arrays.stream(bufferViews).forEach(bufferView -> pattern.applyTo(bufferView));
  }

  /**
   * Apply a pattern to a group of segments.
   *
   * @param pattern the pattern to apply
   * @param segments the segments to apply the pattern to
   */
  public void applyPattern(LEDPattern pattern, Segments... segments) {
    Arrays.stream(segments).forEach(segment -> pattern.applyTo(segment.bufferView));
  }

  /**
   * Fill a group of segments with a given color.
   *
   * @param color the color to fill the segments with
   * @param segments the segments to fill
   */
  public void fill(Color color, Segments... segments) {
    applyPattern(LEDPattern.solid(color), segments);
  }

  /**
   * Clear a group of segments by filling them with black.
   *
   * @param segments the segments to clear
   */
  public void clear(Segments... segments) {
    fill(Color.kBlack, segments);
  }

  /**
   * Creates a number of blocks of a given color, with a given size and space between each block,
   * and stacks them on the provided LEDWriters
   *
   * @param color pixel color
   * @param numBlocks number of blocks to stack
   * @param blockSize size of each block
   * @param blockSpace space between each block
   * @param writers LEDWriters to write to
   */
  public void stackBlocks(
      Color color,
      int numBlocks,
      int blockSize,
      int blockSpace,
      Optional<Color> blockSpaceColor,
      LEDWriter... writers) {
    for (int i = 0; i < numBlocks * (blockSize + blockSpace); i += blockSize + blockSpace) {
      setPixels(i, blockSize, color, writers);
      if (blockSpaceColor.isPresent()) {
        setPixels(i + blockSize, blockSpace, blockSpaceColor.get(), writers);
      }
    }
  }

  /**
   * Sets <i>n</i> LED blocks to the alliance color, starting from the bottom, on both the left and
   * right strips, where <i>n</i> corresponds to the auto mode selected. If no auto is selected, two
   * yellow pixels are lit at the bottom of each strip, and if the alliance inputs disagree, two
   * yellow pixels are lit at the top of each strip.
   */
  public void displayAutoSelection(Color color, int numBlocks, boolean agreement, boolean overlay) {
    if (!overlay) {
      clear(Segments.ALL);
    }
    stackBlocks(
        color,
        numBlocks,
        LedConstants.kLEDsPerBlock,
        LedConstants.kLEDsBetweenBlocks,
        Optional.of(overlay ? null : Color.kBlack),
        leftStrip,
        rightStrip);
    if (!agreement) {
      setPixels(0, 2, Color.kYellow, leftStrip.reversed(), rightStrip.reversed());
    }
    update();
  }

  /**
   * Uses the middle segments to indicate the control mode, and the top and bottom segments to
   * indicate the gamepiece currently possessed by the robot.
   *
   * <p>The middle segments are lit in the coral color if the robot is in coral mode, and in the
   * algae color if the robot is in algae mode. The top and bottom segments are lit in the color of
   * the gamepiece if the robot possesses one, else black.
   *
   * @param isAlgaeMode whether the robot is in algae mode
   * @param gamepiece gamepiece currently possessed by the robot
   */
  public void displayDefaultInfo(boolean isAlgaeMode, Optional<Gamepiece> gamepiece) {
    var modeColor = isAlgaeMode ? LedConstants.algaeColor : LedConstants.coralColor;
    fill(modeColor, Segments.MIDDLE);
    var pieceColor = gamepiece.isPresent() ? gamepiece.get().color : Color.kBlack;
    fill(pieceColor, Segments.TOP_AND_BOTTOM);
    update();
  }

  /**
   * Use LEDs to indicate how to move the robot relative to its current position to reach the target
   * pose. The directions are as follows:
   *
   * <ul>
   *   <li><b>Heading</b> how to face the robot in the correct direction:
   *       <ul>
   *         <li>Use middle segments:
   *         <li>Both white indicates proper heading.
   *         <li>Both cyan means rotate clockwise.
   *         <li>Both magenta means rotate counter-clockwise.
   *       </ul>
   *   <li><b>X</b> Use top segments:
   *       <ul>
   *         <li>White means no change.
   *         <li>Green means move forward.
   *         <li>Red means move backward.
   *       </ul>
   *   <li><b>Y</b> Use bottom segments:
   *       <ul>
   *         <li>White means no change.
   *         <li>Green on one side means move that direction.
   *       </ul>
   * </ul>
   *
   * @param currentPose the robot's current pose
   * @param targetPose the target pose
   */
  public void displayPoseSeek(Pose2d currentPose, Pose2d targetPose) {
    var delta = targetPose.minus(currentPose);
    var theta = delta.getRotation().getDegrees();
    fill(theta == 0 ? Color.kWhite : theta <= 180 ? Color.kMagenta : Color.kCyan, Segments.MIDDLE);

    var x = delta.getTranslation().getX();
    fill(
        (Centimeters.of(x)).baseUnitMagnitude() < 1
            ? Color.kWhite
            : x > 0 ? Color.kGreen : Color.kRed,
        Segments.TOP);

    var y = delta.getTranslation().getY();
    if (Centimeters.of(y).baseUnitMagnitude() < 1) {
      fill(Color.kWhite, Segments.BOTTOM);
    } else {
      fill(Color.kGreen, y > 0 ? Segments.rightBottom : Segments.leftBottom);
      fill(Color.kBlack, y > 0 ? Segments.leftBottom : Segments.rightBottom);
    }
  }

  public void replaceDefaultCommandImmediately(Command command) {
    if (getDefaultCommand() != null) {
      getDefaultCommand().cancel();
    }
    setDefaultCommand(command);
  }

  /**
   * Creates a {@link Command} to run the provided {@link Runnable} action.
   *
   * <p>This method simply wraps the provided action in a {@link Command} that executes the action
   * when it is run.
   *
   * @param action the action to run.
   * @return a new {@link Command} that runs the given action.
   * @see Command#run(Runnable)
   */
  public Command newCommand(Runnable action) {
    return this.run(action);
  }

  /**
   * Create a command to display the default information on the LEDs.
   *
   * @param isAlgeMode whether the robot is in algae mode
   * @param gamepiece gamepiece currently possessed by the robot
   * @return a command to display the default information
   */
  public Command createStandardDisplayCommand(
      BooleanSupplier isAlgeMode, Supplier<Gamepiece> gamepiece) {
    return newCommand(
        () -> displayDefaultInfo(isAlgeMode.getAsBoolean(), Optional.ofNullable(gamepiece.get())));
  }

  /**
   * Create a command to display pose seeking information on the LEDs.
   *
   * @param targetPoseSupplier provides the target pose
   * @param currentPoseSupplier provides the current pose
   * @return a command to display pose seeking information
   */
  public Command createPoseSeekingCommand(
      Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> currentPoseSupplier) {
    return newCommand(() -> displayPoseSeek(currentPoseSupplier.get(), targetPoseSupplier.get()));
  }

  /**
   * Create a pattern to display stacked blocks on the left and right strips.
   *
   * @return a command to display stacked blocks
   */
  public LEDPattern stackedBlocksPattern(
      Color color, int blockSize, int blockSpace, Optional<Color> blockSpaceColor) {
    return (reader, writer) -> {
      stackBlocks(
          color,
          reader.getLength() / (blockSize + blockSpace),
          blockSize,
          blockSpace,
          blockSpaceColor,
          writer);
    };
  }

  /**
   * Create a command to run scrolling blocks on the left and right strips.
   *
   * @return a command to run scrolling blocks
   */
  public LEDPattern stackedBlocksPattern(Color color, int blockSize, int blockSpace) {
    return stackedBlocksPattern(color, blockSize, blockSpace, Optional.of(Color.kBlack));
  }

  /**
   * Displays several bits of information:
   *
   * <ul>
   *   <li>The number of the selected auto and its alliance color. The number and color of the
   *       blocks displayed is determined by the number and alliance color of the selected auto.
   *   <li>Pose-seek info if a pose is provided by the selected auto
   *   <li>A yellow block at the bottom of the strips if no auto is selected
   *   <li>A yellow block at the top of the strips if the configured alliance and the one provided
   *       by the FMS disagree
   * </ul>
   *
   * @param autoOption selected auto
   * @param currentPose current pose of the robot
   * @param agreement whether the configured alliance and the one provided by the FMS agree
   */
  public void displayAutoMode(
      Optional<AutoOption> autoOption, Pose2d currentPose, boolean agreement) {
    /*
     * If we have an auto selected, we need to display its info.  This will
     * be overlaid on top of pose-seek display if its has an
     * initial pose, otherwise, it will overwrite all segments.
     *
     * If no auto is selected we create a display with one yellow block illuminated at
     * the bottom.
     *
     * In either case, the top two pixels on either side will be illuminated
     * yellow is there is disagreement between the configured alliance and
     * the one provided by the FMS.
     */
    autoOption.ifPresentOrElse(
        /*
         * We have an auto
         */
        auto -> {
          var hasInitialPose = auto.getInitialPose().isPresent();
          if (hasInitialPose) {
            displayPoseSeek(currentPose, auto.getInitialPose().get());
          }
          displayAutoSelection(
              auto.getAllianceColor(), auto.getOptionNumber(), agreement, hasInitialPose);
        },
        /*
         * We do not have an auto
         */
        () -> {
          displayAutoSelection(Color.kYellow, 1, agreement, false);
        });
  }

  /**
   * Create a command to run a scrolling rainbow on the left and right strips.
   *
   * @return a command to run a scrolling rainbow
   */
  public Command createScrollingRainbowCommand() {
    Time duration = Seconds.of(0.5);
    LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    LEDPattern scrollingRainbow = rainbow.scrollAtRelativeSpeed(duration.asFrequency());
    return newCommand(() -> applyPattern(scrollingRainbow, leftStrip, rightStrip))
        .withTimeout(duration);
  }

  /**
   * Create a command to display the selected auto option on the LEDs.
   *
   * @param autoSupplier provides the selected auto option
   * @param currentPoseSupplier provides the current pose of the robot
   * @param allianceAgreementSupplier provides whether the configured alliance and the one provided
   *     by the FMS agree
   * @return a command to display the selected auto info
   */
  public Command createAutoOptionDisplayCommand(
      Supplier<Optional<AutoOption>> autoSupplier,
      Supplier<Pose2d> currentPoseSupplier,
      BooleanSupplier allianceAgreementSupplier) {
    return new Command() {
      {
        addRequirements(LEDs.this);
        setName("Auto Mode Display");
      }

      @Override
      public void execute() {
        displayAutoMode(
            autoSupplier.get(),
            currentPoseSupplier.get(),
            allianceAgreementSupplier.getAsBoolean());
      }
    };
  }

  public static AddressableLEDBufferView[] intakeBuffers = {
    // leftStrip,
    // rightStrip
    Segments.leftTop.bufferView.reversed(),
    Segments.rightTop.bufferView.reversed(),
    Segments.leftBottom.bufferView,
    Segments.rightBottom.bufferView
  };

  public static AddressableLEDBufferView[] outtakeBuffers = {
    // leftStrip.reversed(),
    // rightStrip.reversed()
    Segments.leftTop.bufferView,
    Segments.rightTop.bufferView,
    Segments.leftBottom.bufferView.reversed(),
    Segments.rightBottom.bufferView.reversed()
  };

  /**
   * Create a command that runs a roller animation. Blocks will scroll toward the center for intake,
   * and away from the center for outtake.
   *
   * @param isIntakeSupplier whether the rollers are intaking
   * @param colorSupplier the color of the roller animation
   * @return a command to run a roller animation
   */
  public Command createRollerAnimationCommand(
      BooleanSupplier isIntakeSupplier, Supplier<Color> colorSupplier) {
    /**
     * A command that captures the color and appropriate buffers to animate when initialized by the
     * scheduler.
     */
    return new Command() {
      LEDPattern scrollingBlocks;
      AddressableLEDBufferView[] buffers;

      {
        addRequirements(LEDs.this);
        setName("Roller Animation");
      }

      @Override
      public void initialize() {
        var blocks = stackedBlocksPattern(colorSupplier.get(), 3, 2);
        scrollingBlocks = blocks.scrollAtRelativeSpeed(Seconds.of(1).asFrequency());
        buffers = isIntakeSupplier.getAsBoolean() ? intakeBuffers : outtakeBuffers;
        setName(
            String.format(
                "%s: %s %s",
                getName(), buffers == intakeBuffers ? "Intake" : "Outtake", colorSupplier.get()));
      }

      @Override
      public void execute() {
        applyPattern(scrollingBlocks, buffers);
      }
    };
  }

  public Command createAutoSelectionEffectCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'createAutoSelectionEffectCommand'");
  }
}
