package frc.game;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum FeederStation {
  blueRight(new Pose2d(Inches.of(33.51), Inches.of(25.80), new Rotation2d(Degrees.of(54 + 180)))),
  blueLeft(new Pose2d(Inches.of(33.51), Inches.of(291.20), new Rotation2d(Degrees.of(306 + 180)))),
  redRight(new Pose2d(Inches.of(657.37), Inches.of(291.20), new Rotation2d(Degrees.of(234 + 180)))),
  redLeft(new Pose2d(Inches.of(657.37), Inches.of(25.8), new Rotation2d(Degrees.of(126 + 180))));

  private Pose2d pose;

  FeederStation(Pose2d pose) {
    final Transform2d feederStationOffset =
        new Transform2d(Inches.of(-18), Inches.of(0), new Rotation2d(0));
    this.pose = pose.plus(feederStationOffset);
  }

  public Pose2d getPose() {
    return pose;
  }
}
