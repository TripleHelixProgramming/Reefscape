package frc.robot.LEDs;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class LEDs extends SubsystemBase {

  public LEDs() {}

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
    return this.runOnce(
        () -> {
          // TODO: Play an animation when the auto selection changes
        });
  }
}
