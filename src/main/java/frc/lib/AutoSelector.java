package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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

  private Optional<AutoOption> currentAutoOption;
  private DigitalInput[] m_switchPositions;
  private Supplier<Alliance> m_allianceColorSupplier;
  private List<AutoOption> m_autoOptions;
  private EventLoop m_loop = new EventLoop();
  private BooleanEvent m_changedAutoSelection;
  private StructPublisher<Pose2d> initialPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("AutonomousInitialPose", Pose2d.struct)
          .publish();

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

  private Optional<AutoOption> findMatchingOption() {
    return m_autoOptions.stream()
        .filter(o -> o.getColor() == getAllianceColor())
        .filter(o -> o.getOption() == getSwitchPosition())
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

  /**
   * @return Object for binding a command to a change in autonomous mode selection
   */
  public Trigger getChangedAutoSelection() {
    return m_changedAutoSelection.castTo(Trigger::new);
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
    m_loop.poll();
    SmartDashboard.putNumber("AutoSelectorSwitchPosition", getSwitchPosition());

    currentAutoOption.ifPresentOrElse(
        ao -> {
          SmartDashboard.putString("SelectedAutonomousMode", ao.getName());
          ao.getInitialPose()
              .ifPresentOrElse(
                  initialPosePublisher::set, () -> initialPosePublisher.set(new Pose2d()));
        },
        () -> {
          SmartDashboard.putString(
              "SelectedAutonomousMode", "None; no auto mode assigned to this slot");
          initialPosePublisher.set(new Pose2d());
        });
  }
}
