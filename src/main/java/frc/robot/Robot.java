// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import static frc.robot.RobotContainer.getRobotContainer;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.ControllerPatroller;
import frc.robot.Constants.OIConstants;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private int m_usb_check_delay = OIConstants.kUSBCheckNumLoops;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    getRobotContainer();

    // Starts recording to data log
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#logging-joystick-data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    getRobotContainer().periodic();
  }

  @Override
  public void disabledInit() {
    getRobotContainer().disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    // Scan the USB devices. If they change, remap the buttons.

    /*
     * Only check if controllers changed every kUSBCheckNumLoops loops of disablePeriodic().
     * This prevents us from hammering on some routines that cause the RIO to lock up.
     */
    RobotContainer rc = getRobotContainer();
    m_usb_check_delay--;
    if (0 >= m_usb_check_delay) {
      m_usb_check_delay = OIConstants.kUSBCheckNumLoops;
      if (ControllerPatroller.getInstance().controllersChanged()) {
        // Reset the joysticks & button mappings.
        rc.configureButtonBindings();
      }
    }
  }

  @Override
  public void autonomousInit() {
    getRobotContainer().autonomousInit();
    m_autonomousCommand = getRobotContainer().getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    getRobotContainer().teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
