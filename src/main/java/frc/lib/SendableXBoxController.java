package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;

public class SendableXBoxController extends XboxController implements Sendable {

  public SendableXBoxController(int port) {
    super(port);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("XBox");

    builder.addDoubleProperty("LeftXAxis", () -> getLeftX(), null);
    builder.addDoubleProperty("LeftYAxis", () -> getLeftY(), null);
    builder.addDoubleProperty("RightXAxis", () -> getRightX(), null);
    builder.addDoubleProperty("RightYAxis", () -> getRightY(), null);

    builder.addDoubleProperty("LeftTriggerAxis", () -> getLeftTriggerAxis(), null);
    builder.addDoubleProperty("RightTriggerAxis", () -> getRightTriggerAxis(), null);

    builder.addBooleanProperty("A", () -> getAButton(), null);
    builder.addBooleanProperty("B", () -> getBButton(), null);
    builder.addBooleanProperty("X", () -> getXButton(), null);
    builder.addBooleanProperty("Y", () -> getYButton(), null);

    builder.addDoubleProperty("POVAngle", () -> getPOV(), null);

    builder.addBooleanProperty("BackButton", () -> getBackButton(), null);
    builder.addBooleanProperty("StartButton", () -> getStartButton(), null);

    builder.addBooleanProperty("LeftStickButton", () -> getLeftStickButton(), null);
    builder.addBooleanProperty("RightStickButton", () -> getRightStickButton(), null);

    builder.addBooleanProperty("LeftBumper", () -> getLeftBumper(), null);
    builder.addBooleanProperty("RightBumper", () -> getRightBumper(), null);
  }
}
