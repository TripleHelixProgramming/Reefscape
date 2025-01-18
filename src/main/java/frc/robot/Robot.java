package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerPatroller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LEDs.LEDs;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.vision.Vision;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Robot extends TimedRobot {

  private List<AutoOption> m_autoOptions = new ArrayList<>();

  private final PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private final AllianceSelector m_allianceSelector;
  private final AutoSelector m_autoSelector;
  private final Drivetrain m_swerve;
  private final LEDs m_LEDs;
  private final Vision m_vision;
  private final Auto m_auto;

  // private final AutoFactory m_autoFactory;

  private CommandZorroController m_driver;
  private CommandXboxController m_operator;

  private int m_usb_check_delay = OIConstants.kUSBCheckNumLoops;

  private Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

  public Robot() {
    m_allianceSelector = new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);

    m_autoSelector =
        new AutoSelector(
            AutoConstants.kAutonomousModeSelectorPorts,
            m_allianceSelector::getAllianceColor,
            m_autoOptions);

    m_swerve = new Drivetrain(m_allianceSelector::fieldRotated);
    m_auto = new Auto(m_allianceSelector, m_swerve);
    m_LEDs = new LEDs();

    m_vision = new Vision();

    configureButtonBindings();
    configureEventBindings();
    configureAutoOptions();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));
  }

  @Override
  public void robotInit() {
    // Starts recording to data log
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#logging-joystick-data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_swerve.setDefaultCommand(
        new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematics, m_driver.getHID()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    checkVision();
    SmartDashboard.putData(m_driver.getHID());
    SmartDashboard.putData(m_operator.getHID());
    SmartDashboard.putData(m_powerDistribution);
  }

  @Override
  public void disabledInit() {
    m_LEDs.setDefaultCommand(
        m_LEDs.createDisabledCommand(
            m_autoSelector::getSwitchPosition,
            m_allianceSelector::getAllianceColor,
            m_allianceSelector::agreementInAllianceInputs));
  }

  @Override
  public void disabledPeriodic() {
    // Scan the USB devices. If they change, remap the buttons.

    /*
     * Only check if controllers changed every kUSBCheckNumLoops loops of disablePeriodic().
     * This prevents us from hammering on some routines that cause the RIO to lock up.
     */
    m_usb_check_delay--;
    if (0 >= m_usb_check_delay) {
      m_usb_check_delay = OIConstants.kUSBCheckNumLoops;
      if (ControllerPatroller.getInstance().controllersChanged()) {
        // Reset the joysticks & button mappings.
        configureButtonBindings();
      }
    }

    m_allianceSelector.disabledPeriodic();
    m_autoSelector.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    m_autoSelector.scheduleAuto();
    m_LEDs.setDefaultCommand(m_LEDs.createEnabledCommand());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_autoSelector.cancelAuto();
    m_LEDs.setDefaultCommand(m_LEDs.createEnabledCommand());
    m_swerve.resetHeadingOffset();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  public void configureButtonBindings() {

    // Clear any active buttons.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    ControllerPatroller cp = ControllerPatroller.getInstance();

    // We use two different types of controllers - Joystick & XboxController.
    // Create objects of the specific types.
    m_driver = new CommandZorroController(cp.findDriverPort());
    m_operator = new CommandXboxController(cp.findOperatorPort());

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    m_driver.DIn()
        .onTrue(new InstantCommand(() -> m_swerve.setHeadingOffset())
        .ignoringDisable(true));

  }
  // spotless:on

  private void configureOperatorButtonBindings() {}

  private void configureEventBindings() {
    m_autoSelector.getChangedAutoSelection().onTrue(m_LEDs.createChangeAutoAnimationCommand());
  }

  private void configureAutoOptions() {
    m_autoOptions.add(new AutoOption(Alliance.Red, 4));
    m_autoOptions.add(new AutoOption(Alliance.Blue, 1, m_auto.exampleRoutine(), "exampleAuto"));
  }

  /**
   * Gets the current drawn from the Power Distribution Hub by a CAN motor controller, assuming that
   * (PDH port number + 10) = CAN ID
   *
   * @param CANBusPort The CAN ID of the motor controller
   * @return Current in Amps on the PDH channel corresponding to the motor channel
   */
  public double getPDHCurrent(int CANBusPort) {
    return m_powerDistribution.getCurrent(CANBusPort - 10);
  }

  private synchronized StructPublisher<Pose2d> getPose2dPublisher(String name) {
    var publisher = posePublishers.get(name);
    if (publisher == null) {
      publisher = NetworkTableInstance.getDefault().getStructTopic(name, Pose2d.struct).publish();
      posePublishers.put(name, publisher);
    }
    return publisher;
  }

  protected void checkVision() {
    m_vision
        .getPoseEstimates()
        .forEach(
            est -> {
              m_swerve.addVisionMeasurement(
                  est.pose().estimatedPose.toPose2d(), est.pose().timestampSeconds, est.stdev());
              getPose2dPublisher(est.name()).set(est.pose().estimatedPose.toPose2d());
            });
  }
}
