package frc.robot.drivetrain.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveToPoseConstants;

public class PathPlannerToPose {

    public static Command driveToPoseCommand(Pose2d currentPose, Pose2d targetPose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            targetPose.transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kZero)),
            targetPose);
        
        PathConstraints constraints = DriveToPoseConstants.kAlignConstraints;
    
        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints, 
            null, 
            new GoalEndState(0.0, targetPose.getRotation()));
    
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}
