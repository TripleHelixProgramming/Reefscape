// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants.Zorro;
import frc.robot.drivetrain.Drivetrain;

public class ZorroDriveCommand extends DriveCommand {

  Joystick m_controller;

  public ZorroDriveCommand(
      Drivetrain subsystem, SwerveDriveKinematics kinematicsType, Joystick joysticks) {
    super(subsystem, kinematicsType);
    this.m_controller = joysticks;
  }

  @Override
  public double getX() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(Zorro.kRightYAxis), 0.05);
  }

  @Override
  public double getY() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(Zorro.kRightXAxis), 0.05);
  }

  @Override
  public double getTheta() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(Zorro.kLeftXAxis), 0.05);
  }

  @Override
  public boolean fieldRelative() {
    return m_controller.getRawButton(Zorro.kEUp);
  }
}
