package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.SimplePoseControllerConstants;
import frc.robot.drivetrain.Drivetrain;
import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
  public final Drivetrain swerve;
  public final Supplier<Pose2d> targetPoseSupplier;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          SimplePoseControllerConstants.kTranslationP,
          0.0,
          0.0,
          SimplePoseControllerConstants.kTranslationConstraints);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          SimplePoseControllerConstants.kTranslationP,
          0.0,
          0.0,
          SimplePoseControllerConstants.kTranslationConstraints);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          SimplePoseControllerConstants.kTranslationP,
          0.0,
          0.0,
          SimplePoseControllerConstants.kRotationConstraints);

  public DriveToPoseCommand(Drivetrain drivetrain, Supplier<Pose2d> targetPosition) {
    swerve = drivetrain;
    targetPoseSupplier = targetPosition;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    xController.reset(swerve.getPose().getX());
    yController.reset(swerve.getPose().getY());
    thetaController.reset(swerve.getPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    var targetPose = targetPoseSupplier.get();

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(currentPose.getX(), targetPose.getX()),
                yController.calculate(currentPose.getY(), targetPose.getY()),
                thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())),
            swerve.getHeading());

    swerve.setRobotRelativeChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = swerve.getPose();
    var targetPose = targetPoseSupplier.get();
    double translationalError =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double thetaError =
        Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    return translationalError < 0.05 && thetaError < Math.toRadians(3);
  }

  @Override
  public void end(boolean interrupted) {}
}
