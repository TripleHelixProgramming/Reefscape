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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveToPoseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.drivetrain.Drivetrain;
import java.util.Set;
import java.util.function.Supplier;

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
  private final Supplier<Pose2d> currentPoseSupplier;

  public synchronized static PathPlannerToPose getInstance(Drivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
    if (instance == null) {
      instance = new PathPlannerToPose(drivetrain, currentPoseSupplier);
    }
    return instance;
  }

  private PathPlannerToPose(Drivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier) {
    this.drivetrain = drivetrain;
    this.currentPoseSupplier = currentPoseSupplier;
    AutoBuilder.configureCustom(AutoBuilder::followPath, currentPoseSupplier, drivetrain::resetOdometry, true);
  }

  public Command driveTo(Supplier<Pose2d> targetPoseSupplier) {
    var waypoints =
        PathPlannerPath.waypointsFromPoses(
            currentPoseSupplier.get(),
            // endPose.transformBy(new Transform2d(Meters.of(-0.5), Meters.of(0.0),
            // Rotation2d.kZero)),
            targetPoseSupplier.get());
    var constraints = DriveToPoseConstants.kAlignConstraints;
    var path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0.0, targetPoseSupplier.get().getRotation()));

    path.preventFlipping = true;

    // AutoBuilder.configure(currentPose, swerve::resetOdometry, swerve::getChassisSpeeds,
    // swerve::setRobotRelativeChassisSpeedsWithFF, controller, config, false, swerve);
    Supplier<Command> commandSupplier = () -> new FollowPathCommand(
        path,
        currentPoseSupplier,
        drivetrain::getChassisSpeeds,
        drivetrain::setRobotRelativeChassisSpeedsWithFF,
        controller,
        config,
        () -> false,
        drivetrain);

    return new DeferredCommand(commandSupplier, Set.of(drivetrain));
  }
}
