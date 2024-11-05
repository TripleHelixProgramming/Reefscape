package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.ZorroController;
import frc.robot.drivetrain.Drivetrain;

public class ZorroDriveCommand extends DriveCommand {

  ZorroController m_controller;

  public ZorroDriveCommand(
      Drivetrain subsystem, SwerveDriveKinematics kinematicsType, ZorroController joystick) {
    super(subsystem, kinematicsType);
    this.m_controller = joystick;
  }

  @Override
  public double getX() {
    return -MathUtil.applyDeadband(m_controller.getRightYAxis(), 0.05);
  }

  @Override
  public double getY() {
    return -MathUtil.applyDeadband(m_controller.getRightXAxis(), 0.05);
  }

  @Override
  public double getTheta() {
    return -MathUtil.applyDeadband(m_controller.getLeftXAxis(), 0.05);
  }

  @Override
  public boolean fieldRelative() {
    return m_controller.getEUp();
  }
}
