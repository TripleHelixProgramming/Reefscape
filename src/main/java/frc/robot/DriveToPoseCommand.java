package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants.DriveToPoseControllerGains;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.vision.Vision;

public class DriveToPoseCommand extends Command {
    public final Drivetrain swerve;
    public final Vision vision;
    public final Supplier<Pose2d> targetPoseSupplier;

    private final PIDController xController = new PIDController(
        DriveToPoseControllerGains.kTraP, 
        DriveToPoseControllerGains.kTraI, 
        DriveToPoseControllerGains.kTraD);
    private final PIDController yController = new PIDController(
        DriveToPoseControllerGains.kTraP, 
        DriveToPoseControllerGains.kTraI, 
        DriveToPoseControllerGains.kTraD);
    private final PIDController thetaController = new PIDController(
        DriveToPoseControllerGains.kRotP, 
        DriveToPoseControllerGains.kRotI, 
        DriveToPoseControllerGains.kRotD);

    public DriveToPoseCommand(Drivetrain drivetrain, Vision poseEstimator, Supplier<Pose2d> targetPosition) {
        swerve = drivetrain;
        vision = poseEstimator;
        targetPoseSupplier = targetPosition;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        var targetPose = targetPoseSupplier.get();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            yController.calculate(currentPose.getY(), targetPose.getY()),
            thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())),
            swerve.getHeading());

        swerve.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerve.getPose();
        var targetPose = targetPoseSupplier.get();
        double xError = Math.abs(targetPose.getX() - currentPose.getX());
        double yError = Math.abs(targetPose.getY() - currentPose.getY());
        double thetaError = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        return xError < 0.05 && yError < 0.05 && thetaError < Math.toRadians(3);
    }
    
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        swerve.setChassisSpeeds(chassisSpeeds);
    }
}
