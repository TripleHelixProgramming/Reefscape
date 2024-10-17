package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants.Zorro;

public class SendableZorroController extends Joystick implements Sendable {

  public SendableZorroController(int port) {
    super(port);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Zorro");

    builder.addDoubleProperty("LeftXAxis", () -> getRawAxis(Zorro.kLeftXAxis), null);
    builder.addDoubleProperty("LeftYAxis", () -> getRawAxis(Zorro.kLeftYAxis), null);
    builder.addDoubleProperty("LeftDial", () -> getRawAxis(Zorro.kLeftDial), null);
    builder.addDoubleProperty("RightDial", () -> getRawAxis(Zorro.kRightDial), null);
    builder.addDoubleProperty("RightXAxis", () -> getRawAxis(Zorro.kRightXAxis), null);
    builder.addDoubleProperty("RightYAxis", () -> getRawAxis(Zorro.kRightYAxis), null);

    builder.addBooleanProperty("AIn", () -> getRawButton(Zorro.kAIn), null);

    builder.addBooleanProperty("BDown", () -> getRawButton(Zorro.kBDown), null);
    builder.addBooleanProperty("BMid", () -> getRawButton(Zorro.kBMid), null);
    builder.addBooleanProperty("BUp", () -> getRawButton(Zorro.kBUp), null);

    builder.addBooleanProperty("CDown", () -> getRawButton(Zorro.kCDown), null);
    builder.addBooleanProperty("CMid", () -> getRawButton(Zorro.kCMid), null);
    builder.addBooleanProperty("CUp", () -> getRawButton(Zorro.kCUp), null);

    builder.addBooleanProperty("DIn", () -> getRawButton(Zorro.kDIn), null);

    builder.addBooleanProperty("EDown", () -> getRawButton(Zorro.kEDown), null);
    builder.addBooleanProperty("EUp", () -> getRawButton(Zorro.kEUp), null);

    builder.addBooleanProperty("FDown", () -> getRawButton(Zorro.kFDown), null);
    builder.addBooleanProperty("FUp", () -> getRawButton(Zorro.kFUp), null);

    builder.addBooleanProperty("GIn", () -> getRawButton(Zorro.kGIn), null);

    builder.addBooleanProperty("HIn", () -> getRawButton(Zorro.kHIn), null);
  }
}
