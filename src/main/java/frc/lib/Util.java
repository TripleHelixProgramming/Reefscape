package frc.lib;

import java.util.Optional;
import java.util.prefs.Preferences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public interface Util {
  /**
   * Map alliance object to corresponding color.
   *
   * @param alliance the alliance whose color we want
   * @return the appropriate color
   */
  public static Color allianceToColor(Alliance alliance) {
    return alliance == Alliance.Blue ? Color.kBlue : Color.kRed;
  }

  public static boolean nearlyEqual(double a, double b) {
    return Math.abs(a - b) < Math.ulp(1);
  }

  public static void storeTranslation2d(String key, Translation2d xy) {
    Preferences.systemRoot().putDouble(key + ".x", xy.getX());
    Preferences.systemRoot().putDouble(key + ".y", xy.getY());
  }

  public static Optional<Translation2d> loadTranslation2d(String key) {
    var x = Preferences.systemRoot().getDouble(key + ".x", Double.NaN);
    var y = Preferences.systemRoot().getDouble(key + ".y", Double.NaN);
    if (x == Double.NaN || y == Double.NaN) {
      return Optional.empty();
    }
    return Optional.of(new Translation2d(x, y));
  }

  public static void storeRotation2d(String key, Rotation2d w) {
    Preferences.systemRoot().putDouble(key + ".w", w.getDegrees());
  }

  public static Optional<Rotation2d> loadRotation2d(String key) {
    var w = Preferences.systemRoot().getDouble(key + ".w", Double.NaN);
    if (w == Double.NaN) {
      return Optional.empty();
    }
    return Optional.of(new Rotation2d(w));
  }

  public static void storeTransform(String key, Transform2d value) {
    storeTranslation2d(key, value.getTranslation());
    storeRotation2d(key, value.getRotation());
  }

  public static Optional<Transform2d> loadTransform(String key) {
    var translation = loadTranslation2d(key);
    var rotation = loadRotation2d(key);
    if (translation.isEmpty() || rotation.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Transform2d(translation.get(), rotation.get()));
  }

  public static void storePose2d(String key, Pose2d value) {
    storeTranslation2d(key, value.getTranslation() );
    storeRotation2d(key, value.getRotation());
  }

  public static Optional<Pose2d> loadPose2d(String key) {
    var translation = loadTranslation2d(key);
    var rotation = loadRotation2d(key);
    if (translation.isEmpty() || rotation.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Pose2d(translation.get(), rotation.get()));
  }
}
