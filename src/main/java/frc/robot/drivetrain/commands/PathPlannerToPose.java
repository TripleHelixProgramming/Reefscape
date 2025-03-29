package frc.robot.drivetrain.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;
import frc.robot.drivetrain.Drivetrain;

public class PathPlannerToPose {
    static ModuleConfig modConfig = new ModuleConfig(Meters.of(0.05), DriveConstants.kMaxDriveToPoseTranslationVelocity, 0.6, DCMotor.getNeoVortex(4), ModuleConstants.kDriveGearRatio, Current.ofBaseUnits(80.0, Amps), 4);
    static RobotConfig config = new RobotConfig(Kilograms.of(52), MomentOfInertia.ofBaseUnits(6.0, KilogramSquareMeters), modConfig, DriveConstants.kTrackWidth);

    public static Command driveToPoseCommand(Drivetrain swerve, Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {

        var pose = currentPose.get();
        var endPose = targetPose.get();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            pose,
            endPose.transformBy(new Transform2d(Meters.of(-0.5), Meters.of(0.0), Rotation2d.kZero)),
            endPose);
        
        PathConstraints constraints = DriveToPoseConstants.kAlignConstraints;
    
        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints, 
            null, 
            new GoalEndState(0.0, endPose.getRotation()));
    
        path.preventFlipping = true;

        // AutoBuilder.configure(currentPose, swerve::resetOdometry, swerve::getChassisSpeeds, swerve::setFieldRelativeChassisSpeeds, new PPHolonomicDriveController(new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(1.0, 0.0, 0.0)), config, false, swerve);

        AutoBuilder.configureCustom(AutoBuilder::followPath, currentPose, swerve::resetOdometry, true);

        return AutoBuilder.followPath(path);
    }
}
