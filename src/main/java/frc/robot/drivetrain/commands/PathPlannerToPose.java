package frc.robot.drivetrain.commands;

import java.util.Set;
import java.util.function.Supplier;

// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.PoseLogger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants.TrajectoryFollowingConstants;
import frc.robot.drivetrain.Drivetrain;

/**
 * A singleton class that provides a command to drive from the robot's current pose to a target
 * pose.
 *
 * <p>PathPlannerToPose is a singleton class because it configures AutoBuiler.
 */
public class PathPlannerToPose {

  private static PathPlannerToPose instance;

  private final Drivetrain drivetrain;

  /**
   * Get the singleton instance of PathPlannerToPose.
   *
   * @param drivetrain the drivetrain to use
   * @return the singleton instance of PathPlannerToPose
   */
  public static synchronized PathPlannerToPose getInstance(Drivetrain drivetrain) {
    if (instance == null) {
      instance = new PathPlannerToPose(drivetrain);
    } else if (instance.drivetrain != drivetrain) {
      throw new IllegalStateException(
          "PathPlannerToPose already initialized with different drivetrain");
    }
    return instance;
  }

  private PathPlannerToPose(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // AutoBuilder.configureCustom(
    //     AutoBuilder::followPath, drivetrain::getPose, drivetrain::resetOdometry, true);
  }

  /**
   * Create a command to drive to the current target pose from the current robot pose.
   *
   * @param targetPoseSupplier supplies the target pose at which to arrive
   * @return the command to drive to the target pose provided by the supplier
   */
  private FollowPathCommand createCommand(Supplier<Pose2d> targetPoseSupplier) {
    var targetPose = targetPoseSupplier.get();
    var fromPose = drivetrain.getPose();
    PoseLogger.getDefault().publish("FollowPath.targetPose", targetPose);
    PoseLogger.getDefault().publish("FollowPath.fromPose", fromPose);
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            fromPose,
            // targetPose.transformBy(new Transform2d(Meters.of(-0.5), Meters.of(0.0),
            // Rotation2d.kZero)),
            targetPose);
    var constraints = TrajectoryFollowingConstants.kPathFollowingConstraints;
    var path =
        new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(0, fromPose.getRotation()),
            new GoalEndState(0.0, targetPose.getRotation()));

    for (int i = 0; i < path.getPathPoses().size(); ++i) {
      PoseLogger.getDefault().publish("trajectory."+i, path.getPathPoses().get(i));
    }

    PathFollowingController controller =
        new PPHolonomicDriveController(
            new PIDConstants(TrajectoryFollowingConstants.kTranslationP, 0.0, 0.0),
            new PIDConstants(TrajectoryFollowingConstants.kRotationP, 0.0, 0.0));

    path.preventFlipping = true;

    return new FollowPathCommand(
        path,
        drivetrain::getPose,
        drivetrain::getChassisSpeeds,
        drivetrain::setRobotRelativeChassisSpeedsWithFF,
        controller,
        DriveConstants.kRobotConfig,
        () -> false,
        drivetrain);
  }

  /**
   * Create a command that will drive to the target pose. Both the target pose and the robot pose
   * will be polled at the time the command is executed.
   *
   * @param targetPoseSupplier
   * @return the command to drive to the target pose provided by the supplier
   */
  public Command driveTo(Supplier<Pose2d> targetPoseSupplier) {
    return new DeferredCommand(() -> createCommand(targetPoseSupplier), Set.of(drivetrain));
  }
}
