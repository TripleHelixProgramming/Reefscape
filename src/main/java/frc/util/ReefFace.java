package frc.util;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;

public enum ReefFace {
  BLUE_AB (DriveConstants.blueAB),
  BLUE_CD (DriveConstants.blueCD),
  BLUE_EF (DriveConstants.blueEF),
  BLUE_GH (DriveConstants.blueGH),
  BLUE_IJ (DriveConstants.blueIJ),
  BLUE_KL (DriveConstants.blueKL),
  RED_AB (DriveConstants.redAB),
  RED_CD (DriveConstants.redCD),
  RED_EF (DriveConstants.redEF),
  RED_GH (DriveConstants.redGH),
  RED_IJ (DriveConstants.redIJ),
  RED_KL (DriveConstants.redKL);

  public final Pose2d centerPose;
  public final Pose2d leftPolePose;
  public final Pose2d rightPolePose;

  ReefFace(Pose2d pose) {
    this.centerPose = pose;
    this.leftPolePose = pose.transformBy(new Transform2d(new Translation2d(0, -DriveConstants.pipeSpacing.in(Inches)/2), Rotation2d.kZero));
    this.rightPolePose = pose.transformBy(new Transform2d(new Translation2d(0, +DriveConstants.pipeSpacing.in(Inches)/2), Rotation2d.kZero));
  }

  /**
   * Get the face on the red reef into whose sector the specified pose falls.
   * @param atPose the pose for which we want to know the reef sector
   * @return the reef face
   */
  public static ReefFace getRedReefFace(Pose2d atPose) {
    var sector = Util.getReefSector(DriveConstants.redReefCenter, atPose);
    return values()[sector + RED_AB.ordinal()];
  }

  /**
   * Get the face on the blue reef into whose sector the specified pose falls.
   * @param atPose the pose for which we want to know the reef sector
   * @return the reef face
   */
  public static ReefFace getBlueReefFace(Pose2d atPose) {
    var sector = Util.getReefSector(DriveConstants.blueReefCenter, atPose);
    return values()[sector];
  }
}