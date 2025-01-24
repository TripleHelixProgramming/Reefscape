package frc.lib;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoOption {
  private final Alliance allicanceColor;
  private final int switchNumber;
  private final Supplier<AutoRoutine> autoSupplier;
  private final String name;
  private AutoRoutine autoRoutine;

  /**
   * Constructs a selectable autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   * @param autoSupplier Supplies command which runs the autonomous mode
   * @param name The name of the autonomous mode option
   */
  public AutoOption(Alliance color, int option, Supplier<AutoRoutine> autoSupplier, String name) {
    this.allicanceColor = color;
    this.switchNumber = option;
    this.autoSupplier = autoSupplier;
    this.name = name;
  }

  /**
   * Constructs a null autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   */
  public AutoOption(Alliance color, int option) {
    this(color, option, null, "empty");
  }

  /**
   * @return Alliance for which the option is valid
   */
  public Alliance getColor() {
    return this.allicanceColor;
  }

  /**
   * @return Selector switch index for which the option is valid
   */
  public int getOption() {
    return this.switchNumber;
  }

  /**
   * @return The command which runs the selected autonomous mode
   */
  public synchronized Optional<Command> getAutoCommand() {
    if (autoRoutine == null) {
      if (autoSupplier == null) {
        return Optional.empty();
      }
      autoRoutine = autoSupplier.get();
    }
    return Optional.of(autoRoutine.cmd());
  }

  public String getName() {
    return this.name;
  }
}
