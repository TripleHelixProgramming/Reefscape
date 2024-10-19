// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.ControllerPatroller;
import frc.lib.SendableZorroController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.Zorro;
import frc.robot.autos.ExampleAuto;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import java.util.List;

public class Robot extends TimedRobot {

  private List<AutoOption> m_autoOptions;

  private final PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private final AllianceSelector m_allianceSelector;
  private final AutoSelector m_autoSelector;
  private final Drivetrain m_swerve;

  private SendableZorroController m_driver;
  private XboxController m_operator;

  private int m_usb_check_delay = OIConstants.kUSBCheckNumLoops;

  public Robot() {
    configureButtonBindings();
    configureDefaultCommands();
    configureAutoOptions();

    m_allianceSelector = new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);
    m_autoSelector =
        new AutoSelector(
            AutoConstants.kAutonomousModeSelectorPorts,
            m_allianceSelector::getAllianceColor,
            m_autoOptions);

    m_swerve = new Drivetrain(m_allianceSelector::fieldRotated);

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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(m_driver);
    SmartDashboard.putData(m_operator);
    SmartDashboard.putData(m_powerDistribution);
  }

  @Override
  public void disabledInit() {}

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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_autoSelector.cancelAuto();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  private void configureDefaultCommands() {
    m_swerve.setDefaultCommand(
        new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematics, m_driver));
  }

  public void configureButtonBindings() {

    // Clear any active buttons.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    ControllerPatroller cp = ControllerPatroller.getInstance();

    // We use two different types of controllers - Joystick & XboxController.
    // Create objects of the specific types.
    m_driver = new SendableZorroController(cp.findDriverPort());
    m_operator = new XboxController(cp.findOperatorPort());

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    new JoystickButton(m_driver, Zorro.kHIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading())
        .ignoringDisable(true));

  }
  // spotless:on

  private void configureOperatorButtonBindings() {}

  private void configureAutoOptions() {
    m_autoOptions.add(new AutoOption(Alliance.Red, 1, new ExampleAuto()));
    m_autoOptions.add(new AutoOption(Alliance.Blue, 1));
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
}
