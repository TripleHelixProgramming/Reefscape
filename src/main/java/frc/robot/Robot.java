package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.auto.ExampleAuto;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DriveToPoseCommand;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.vision.Vision;
import java.util.HashMap;
import java.util.Map;

public class Robot extends TimedRobot {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private final AllianceSelector allianceSelector =
      new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);
  private final AutoSelector autoSelector =
      new AutoSelector(
          AutoConstants.kAutonomousModeSelectorPorts, allianceSelector::getAllianceColor);
  private final Drivetrain swerve = new Drivetrain(allianceSelector::fieldRotated);
  private final LEDs leds = new LEDs();
  private final Vision vision = new Vision();
  private CommandZorroController driver;
  private CommandXboxController operator;
  private int usbCheckDelay = OIConstants.kUSBCheckNumLoops;
  private Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

  private Pose2d[] targetPoses = {
    new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(0.0)),
    new Pose2d(1.0, 5.0, Rotation2d.fromDegrees(0.0))
  };

  public Robot() {
    configureButtonBindings();
    configureEventBindings();
    configureAutoOptions();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));
  }

  @Override
  public void robotInit() {
    // Starts recording to data log
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#logging-joystick-data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    swerve.setDefaultCommand(
        new ZorroDriveCommand(swerve, DriveConstants.kDriveKinematics, driver.getHID()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    checkVision();
    SmartDashboard.putData(driver.getHID());
    SmartDashboard.putData(operator.getHID());
    SmartDashboard.putData(powerDistribution);
  }

  @Override
  public void disabledInit() {
    leds.setDefaultCommand(
        leds.createDisabledCommand(
            autoSelector::getSwitchPosition,
            allianceSelector::getAllianceColor,
            allianceSelector::agreementInAllianceInputs));
  }

  @Override
  public void disabledPeriodic() {
    // Scan the USB devices. If they change, remap the buttons.

    /*
     * Only check if controllers changed every kUSBCheckNumLoops loops of disablePeriodic().
     * This prevents us from hammering on some routines that cause the RIO to lock up.
     */
    usbCheckDelay--;
    if (0 >= usbCheckDelay) {
      usbCheckDelay = OIConstants.kUSBCheckNumLoops;
      if (ControllerPatroller.getInstance().controllersChanged()) {
        // Reset the joysticks & button mappings.
        configureButtonBindings();
      }
    }

    allianceSelector.disabledPeriodic();
    autoSelector.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    autoSelector.scheduleAuto();
    leds.setDefaultCommand(leds.createEnabledCommand());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    autoSelector.cancelAuto();
    leds.setDefaultCommand(leds.createEnabledCommand());
    swerve.resetHeadingOffset();
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
    driver = new CommandZorroController(cp.findDriverPort());
    operator = new CommandXboxController(cp.findOperatorPort());

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    driver.DIn()
        .onTrue(new InstantCommand(() -> swerve.setHeadingOffset())
        .ignoringDisable(true));

    driver.AIn()
        .whileTrue(new DriveToPoseCommand(swerve, vision, () -> getNearestPose(swerve.getPose(), targetPoses)));

  }
  // spotless:on

  private void configureOperatorButtonBindings() {}

  private void configureEventBindings() {
    autoSelector.getChangedAutoSelection().onTrue(leds.createChangeAutoAnimationCommand());
  }

  private void configureAutoOptions() {
    autoSelector.addAuto(new AutoOption(Alliance.Red, 4));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 1, new ExampleAuto(swerve)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 2));
  }

  /**
   * Gets the current drawn from the Power Distribution Hub by a CAN motor controller, assuming that
   * (PDH port number + 10) = CAN ID
   *
   * @param CANBusPort The CAN ID of the motor controller
   * @return Current in Amps on the PDH channel corresponding to the motor channel
   */
  public double getPDHCurrent(int CANBusPort) {
    return powerDistribution.getCurrent(CANBusPort - 10);
  }

  public Pose2d getNearestPose(Pose2d currentPose, Pose2d[] targetPoses) {
    Pose2d closestPose = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
    double minDistance = Double.MAX_VALUE;

    for (Pose2d targetPose : targetPoses) {
      double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        closestPose = targetPose;
      }
    }
    return closestPose;
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
    vision
        .getPoseEstimates()
        .forEach(
            est -> {
              swerve.addVisionMeasurement(
                  est.pose().estimatedPose.toPose2d(), est.pose().timestampSeconds, est.stdev());
              getPose2dPublisher(est.name()).set(est.pose().estimatedPose.toPose2d());
            });
  }
}
