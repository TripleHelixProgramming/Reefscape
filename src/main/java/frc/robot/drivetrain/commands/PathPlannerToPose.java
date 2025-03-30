package frc.robot.drivetrain.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.PoseLogger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.drivetrain.Drivetrain;
import java.util.Set;
import java.util.function.Supplier;

/**
 * A singleton class that provides a command to drive from the robot's current pose to a target
 * pose.
 *
 * <p>PathPlannerToPose is a singleton class because it configures AutoBuiler.
 */
public class PathPlannerToPose {
  private static ModuleConfig modConfig =
      new ModuleConfig(
          Meters.of(0.05),
          DriveConstants.kMaxDriveToPoseTranslationVelocity,
          0.6,
          DCMotor.getNeoVortex(4),
          ModuleConstants.kDriveGearRatio,
          Current.ofBaseUnits(80.0, Amps),
          4);
  private static RobotConfig config =
      new RobotConfig(
          Kilograms.of(52),
          MomentOfInertia.ofBaseUnits(6.0, KilogramSquareMeters),
          modConfig,
          DriveConstants.kTrackWidth);

  private static PathFollowingController controller =
      new PPHolonomicDriveController(
          new PIDConstants(1.0, 0.0, 0.0), new PIDConstants(3.0, 0.0, 0.0));

  private static PathPlannerToPose instance;

  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> robotPoseSupplier;

  /**
   * Get the singleton instance of PathPlannerToPose.
   *
   * @param drivetrain the drivetrain to use
   * @param robotPoseSupplier the supplier of the current robot pose
   * @return the singleton instance of PathPlannerToPose
   */
  public static synchronized PathPlannerToPose getInstance(
      Drivetrain drivetrain, Supplier<Pose2d> robotPoseSupplier) {
    if (instance == null) {
      instance = new PathPlannerToPose(drivetrain, robotPoseSupplier);
    } else if (instance.drivetrain != drivetrain
        || instance.robotPoseSupplier != robotPoseSupplier) {
      throw new IllegalStateException(
          "PathPlannerToPose already initialized with different drivetrain or currentPoseSupplier");
    }
    return instance;
  }

  private PathPlannerToPose(Drivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
    this.drivetrain = drivetrain;
    this.robotPoseSupplier = currentPoseSupplier;
    AutoBuilder.configureCustom(
        AutoBuilder::followPath, currentPoseSupplier, drivetrain::resetOdometry, true);
  }

  /**
   * Create a command to drive to the current target pose from the current robot pose.
   *
   * @param targetPoseSupplier supplies the target pose at which to arrive
   * @return the command to drive to the target pose provided by the supplier
   */
  private FollowPathCommand createCommand(Supplier<Pose2d> targetPoseSupplier) {
    var targetPose = targetPoseSupplier.get();
    var fromPose = robotPoseSupplier.get();
    PoseLogger.getDefault().publish("FollowPath.targetPose", targetPose);
    PoseLogger.getDefault().publish("FollowPath.fromPose", fromPose);
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            fromPose,
            // endPose.transformBy(new Transform2d(Meters.of(-0.5), Meters.of(0.0),
            // Rotation2d.kZero)),
            targetPose);
    var constraints = DriveToPoseConstants.kAlignConstraints;
    var path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0.0, targetPose.getRotation()));

    path.preventFlipping = true;

    return new FollowPathCommand(
        path,
        robotPoseSupplier,
        drivetrain::getChassisSpeeds,
        drivetrain::setRobotRelativeChassisSpeedsWithFF,
        controller,
        config,
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
