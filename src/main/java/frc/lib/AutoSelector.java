package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoSelector implements Supplier<Optional<AutoOption>> {

  private Optional<AutoOption> currentAutoOption;
  private DigitalInput[] switchPositions;
  private Supplier<Alliance> allianceColorSupplier;
  private List<AutoOption> autoOptions = new ArrayList<>();
  private EventLoop eventLoop = new EventLoop();
  private BooleanEvent autoSelectionChanged;

  /**
   * Constructs an autonomous selector switch
   *
   * @param ports An array of DIO ports for selecting an autonomous mode
   * @param allianceColorSupplier A method that supplies the current alliance color
   * @param autoOptions An array of autonomous mode options
   */
  public AutoSelector(int[] ports, Supplier<Alliance> allianceColorSupplier) {
    this.allianceColorSupplier = allianceColorSupplier;

    switchPositions = new DigitalInput[ports.length];
    for (int i = 0; i < ports.length; i++) {
      switchPositions[i] = new DigitalInput(ports[i]);
    }

    autoSelectionChanged = new BooleanEvent(eventLoop, () -> updateAuto());

    PoseLogger.getDefault()
        .monitor(
            "autonomousStart",
            () -> {
              return getInitialPose();
            });
  }

  public void addAuto(AutoOption newAuto) {
    autoOptions.add(newAuto);
  }

  /**
   * @return The position of the autonomous selection switch
   */
  public int getRotarySwitchPosition() {
    for (int i = 0; i < switchPositions.length; i++) {
      if (!switchPositions[i].get()) {
        return i + 1;
      }
    }
    return 0; // failure of the physical switch
  }

  public int getBinarySwitchPosition() {
    int sum = 0;
    for (int i = 0; i < switchPositions.length; i++) {
      if (!switchPositions[i].get()) {
        sum += (1 << (i));
      }
    }
    return sum;
  }

  private Alliance getAlliance() {
    return allianceColorSupplier.get();
  }

  private Optional<AutoOption> findMatchingOption() {
    return autoOptions.stream()
        .filter(o -> o.getAlliance() == getAlliance())
        .filter(o -> o.getOptionNumber() == getBinarySwitchPosition())
        .findFirst();
  }

  private boolean updateAuto() {
    var newAutoOption = findMatchingOption();
    if (newAutoOption.equals(currentAutoOption)) {
      return false;
    }
    currentAutoOption = newAutoOption;
    return true;
  }

  @Override
  public Optional<AutoOption> get() {
    return currentAutoOption;
  }

  /**
   * @return Object for binding a command to a change in autonomous mode selection
   */
  public Trigger getChangedAutoSelection() {
    return autoSelectionChanged.castTo(Trigger::new);
  }

  /** Schedules the command corresponding to the selected autonomous mode */
  public void scheduleAuto() {
    currentAutoOption.ifPresent(ao -> ao.getAutoCommand().ifPresent(Command::schedule));
  }

  /** Deschedules the command corresponding to the selected autonomous mode */
  public void cancelAuto() {
    currentAutoOption.ifPresent(ao -> ao.getAutoCommand().ifPresent(Command::cancel));
  }

  public void disabledPeriodic() {
    eventLoop.poll();
    SmartDashboard.putNumber("AutoSelectorSwitchPosition", getBinarySwitchPosition());

    SmartDashboard.putBoolean("AutoSelectorDIOPort0", !switchPositions[0].get());
    SmartDashboard.putBoolean("AutoSelectorDIOPort1", !switchPositions[1].get());
    SmartDashboard.putBoolean("AutoSelectorDIOPort2", !switchPositions[2].get());

    currentAutoOption.ifPresentOrElse(
        ao -> {
          SmartDashboard.putString("SelectedAutonomousMode", ao.getName());
        },
        () -> {
          SmartDashboard.putString(
              "SelectedAutonomousMode", "None; no auto mode assigned to this slot");
        });
  }

  public Optional<Pose2d> getInitialPose() {
    return currentAutoOption.isPresent()
        ? currentAutoOption.get().getInitialPose()
        : Optional.empty();
  }
}
