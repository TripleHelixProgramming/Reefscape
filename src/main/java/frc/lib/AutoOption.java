package frc.lib;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import choreo.auto.AutoRoutine;
import java.util.Optional;

public class AutoOption {
  private Alliance m_color;
  private int m_option;
  private Optional<AutoRoutine> m_auto;
  private String name;

  /**
   * Constructs a selectable autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   * @param auto Command which runs the autonomous mode
   */
  public AutoOption(Alliance color, int option, AutoRoutine auto, String name) {
    this.m_color = color;
    this.m_option = option;
    this.m_auto = Optional.of(auto);
    this.name = name;
  }

  /**
   * Constructs a null autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   */
  public AutoOption(Alliance color, int option) {
    this.m_color = color;
    this.m_option = option;
    this.m_auto = Optional.empty();
  }

  /**
   * @return Alliance for which the option is valid
   */
  public Alliance getColor() {
    return this.m_color;
  }

  /**
   * @return Selector switch index for which the option is valid
   */
  public int getOption() {
    return this.m_option;
  }

  /**
   * @return The command which runs the selected autonomous mode
   */
  public Optional<Command> getAutoCommand() {
    return Optional.of(this.m_auto.get().cmd());
  }

  public String getName() {
    return this.name;
  }
}
