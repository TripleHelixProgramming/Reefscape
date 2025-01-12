package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.Drivetrain;

public abstract class DriveCommand extends Command {

  private LinearVelocity xDot;
  private LinearVelocity yDot;
  private AngularVelocity thetaDot;

  // used to swap control locations
  SwerveDriveKinematics kinematics;

  // The subsystem the command runs on
  public final Drivetrain drivetrain;

  public DriveCommand(Drivetrain subsystem, SwerveDriveKinematics kinematics) {
    drivetrain = subsystem;
    addRequirements(drivetrain);

    this.kinematics = kinematics;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xDot = DriveConstants.kMaxTranslationalVelocity.times(getX());
    yDot = DriveConstants.kMaxTranslationalVelocity.times(getY());
    thetaDot = DriveConstants.kMaxRotationalVelocity.times(getTheta());

    SmartDashboard.putBoolean("fieldRelative", fieldRelative());
    SmartDashboard.putBoolean("rotateField", drivetrain.fieldRotatedSupplier().getAsBoolean());

    drivetrain.setChassisSpeeds(
        fieldRelative() ? getFieldRelativeChassisSpeeds() : getRobotRelativeChassisSpeeds(),
        kinematics);
  }

  private ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return new ChassisSpeeds(xDot.unaryMinus(), yDot.unaryMinus(), thetaDot);
  }

  // spotless:off
  private ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return drivetrain.fieldRotatedSupplier().getAsBoolean()
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xDot, yDot, thetaDot, drivetrain.getHeading().rotateBy(new Rotation2d(Math.PI)))
        : ChassisSpeeds.fromFieldRelativeSpeeds(
            xDot, yDot, thetaDot, drivetrain.getHeading());
  }
  // spotless:on

  /**
   * @return The input to the drive controller in the x axis, range [-1, 1]
   */
  public abstract double getX();

  /**
   * @return The input to the drive controller in the y axis, range [-1, 1]
   */
  public abstract double getY();

  /**
   * @return The input to the drive controller in the theta axis, range [-1, 1]
   */
  public abstract double getTheta();

  /**
   * @return Boolean representing whether to drive in field-relative mode
   */
  public abstract boolean fieldRelative();
}
