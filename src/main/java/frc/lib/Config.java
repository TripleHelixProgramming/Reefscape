package frc.lib;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import java.util.Optional;

public class Config {

  private static Config defaultConfig = new Config(Preferences.getString("defaultConfig", "dcmp"));

  public static Config getDefault() {
    return defaultConfig;
  }

  private final String name;

  public Config(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public String prefix(String key) {
    return name + "." + key;
  }

  public void storeDouble(String key, double value) {
    Preferences.setDouble(prefix(key), value);
  }

  public Optional<Double> loadDouble(String key) {
    var value = Preferences.getDouble(prefix(key), Double.NaN);
    if (Double.isNaN(value)) {
      return Optional.empty();
    }
    return Optional.of(value);
  }

  public void storeTranslation2d(String key, Translation2d xy) {
    storeDouble(key + ".x", xy.getX());
    storeDouble(key + ".y", xy.getY());
  }

  public Optional<Translation2d> loadTranslation2d(String key) {
    var x = loadDouble(key + ".x");
    var y = loadDouble(key + ".y");
    if (x.isEmpty() || y.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Translation2d(x.get(), y.get()));
  }

  public void storeRotation2d(String key, Rotation2d w) {
    storeDouble(key + ".w", w.getDegrees());
  }

  public Optional<Rotation2d> loadRotation2d(String key) {
    var w = loadDouble(key + ".w");
    if (w.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Rotation2d(Degrees.of(w.get())));
  }

  public void storeTransform(String key, Transform2d value) {
    storeTranslation2d(key, value.getTranslation());
    storeRotation2d(key, value.getRotation());
  }

  public Optional<Transform2d> loadTransform(String key) {
    var translation = loadTranslation2d(key);
    var rotation = loadRotation2d(key);
    if (translation.isEmpty() || rotation.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Transform2d(translation.get(), rotation.get()));
  }

  public void storePose2d(String key, Pose2d value) {
    storeTranslation2d(key, value.getTranslation());
    storeRotation2d(key, value.getRotation());
  }

  public Optional<Pose2d> loadPose2d(String key) {
    var translation = loadTranslation2d(key);
    var rotation = loadRotation2d(key);
    if (translation.isEmpty() || rotation.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Pose2d(translation.get(), rotation.get()));
  }
}
