package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import frc.game.FeederStation;
import frc.game.Reef;

public final class Constants {

  public static final class MotorConstants {
    public static final class NEOConstants {
      public static final AngularVelocity kFreeSpeed = RPM.of(5676);
      public static final int kDefaultCurrentLimit = 80;
    }

    public static final class NEO550Constants {
      public static final AngularVelocity kFreeSpeed = RPM.of(11000);
      public static final int kDefaultCurrentLimit = 30;
    }

    public static final class NEOVortexConstants {
      public static final int kDefaultCurrentLimit = 80;
    }
  }

  public static final class VisionConstants {
    // Define the standard deviations for the pose estimator, which determine how fast the pose
    // estimate converges to the vision measurement. This should depend on the vision measurement
    // noise and how many or how frequently vision measurements are applied to the pose estimator.
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class RobotConstants {
    public static final double kNominalVoltage = 12.0;
    public static final Time kPeriod = Seconds.of(TimedRobot.kDefaultPeriod);
  }

  public static final class DriveConstants {

    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final class MotorControllers {
      public static final int kRearRightDriveMotorPort = 10;
      public static final int kFrontRightDriveMotorPort = 20;
      public static final int kFrontLeftDriveMotorPort = 28;
      public static final int kRearLeftDriveMotorPort = 12;

      public static final int kRearRightTurningMotorPort = 11;
      public static final int kFrontRightTurningMotorPort = 21;
      public static final int kFrontLeftTurningMotorPort = 29;
      public static final int kRearLeftTurningMotorPort = 13;
    }

    public static final class AbsoluteEncoders {
      public static final int kRearRightTurningEncoderPort = 31;
      public static final int kFrontRightTurningEncoderPort = 33;
      public static final int kFrontLeftTurningEncoderPort = 43;
      public static final int kRearLeftTurningEncoderPort = 45;

      public static final String kAbsEncoderMagnetOffsetKey = "AbsEncoderMagnetOffsetKey";
      public static final double kDefaultAbsEncoderMagnetOffset = 0.0;
    }

    // Distance between centers of right and left wheels on robot
    public static final Distance kTrackWidth = Inches.of(21);

    // Distance between front and rear wheels on robot
    public static final Distance kWheelBase = Inches.of(27);

    // Robot radius
    public static final double kRadius = 0.423;

    public static final LinearVelocity kMaxTranslationalVelocity =
        MetersPerSecond.of(4.0); // max 4.5
    public static final AngularVelocity kMaxRotationalVelocity =
        RadiansPerSecond.of(5.0); // max 5.0
    public static final LinearVelocity kMinTranslationVelocity = MetersPerSecond.of(1.0);

    // The locations for the modules must be relative to the center of the robot.
    // Positive x values represent moving toward the front of the robot
    // Positive y values represent moving toward the left of the robot.
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase.times(0.5), kTrackWidth.times(0.5)), // front left
            new Translation2d(kWheelBase.times(0.5), kTrackWidth.times(-0.5)), // front right
            new Translation2d(kWheelBase.times(-0.5), kTrackWidth.times(0.5)), // rear left
            new Translation2d(kWheelBase.times(-0.5), kTrackWidth.times(-0.5)) // rear right
            );

    public static final Pose2d[] kReefTargetPoses = {
      new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(0.0)),
      new Pose2d(1.0, 5.0, Rotation2d.fromDegrees(0.0)),
      Reef.Face.blueAB.getCenterPose(),
      Reef.Face.blueCD.getCenterPose(),
      Reef.Face.blueEF.getCenterPose(),
      Reef.Face.blueGH.getCenterPose(),
      Reef.Face.blueIJ.getCenterPose(),
      Reef.Face.blueKL.getCenterPose(),
      Reef.Face.redAB.getCenterPose(),
      Reef.Face.redCD.getCenterPose(),
      Reef.Face.redEF.getCenterPose(),
      Reef.Face.redGH.getCenterPose(),
      Reef.Face.redIJ.getCenterPose(),
      Reef.Face.redKL.getCenterPose(),
      FeederStation.blueRight.getPose(),
      FeederStation.blueLeft.getPose(),
      FeederStation.redRight.getPose(),
      FeederStation.redLeft.getPose()
    };
  }

  public static final class ModuleConstants {
    public static final class DriveControllerGains {
      public static final double kP = 0.1; // 2023 Competition Robot
      public static final double kI = 0.0; // 2023 Competition Robot
      public static final double kD = 0.0; // 2023 Competition Robot
      public static final double kFF = 0.255; // 2023 Competition Robot
    }

    public static final class TurningControllerGains {
      public static final double kP = 1.5; // 1.5;
      public static final double kI = 0.0; // 2023 Competition Robot
      public static final double kD = 0.0; // 2023 Competition Robot
    }

    public static final class DriveToPoseControllerGains {
      public static final double kTraP = 2.0;
      public static final double kTraI = 0.0;
      public static final double kTraD = 0.0;
      public static final double kRotP = 3.0;
      public static final double kRotI = 0.0;
      public static final double kRotD = 0.0;
    }

    // Not adjusted
    // public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0.05 * Math.PI;
    // public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared =
    //    0.05 * Math.PI;

    // public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254,
    // 0.137);

    public static final double kWheelDiameterMeters = 0.047; // 1.87 in. avg diamter 2024 bot

    // By default, the drive encoder in position mode measures rotations at the drive motor
    // Convert to meters at the wheel
    public static final double kDriveGearRatio = 6.75; // 2023 Competion Robot
    public static final double kDrivePositionConversionFactor =
        (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // By default, the drive encoder in velocity mode measures RPM at the drive motor
    // Convert to meters per second at the wheel
    public static final double kDriveVelocityConversionFactor =
        kDrivePositionConversionFactor / 60.0;

    // By default, the turn encoder in position mode measures rotations at the turning motor
    // Convert to rotations at the module azimuth
    public static final double kTurnGearRatio = 12.8; // 2023 Competion Robot
    public static final double kTurnPositionConversionFactor = 1.0 / kTurnGearRatio;
  }

  public static final class OIConstants {
    public static final int kUSBCheckNumLoops = 40;

    public static final String kXbox = "XBOX";
    public static final String kPS4 = "P";
    public static final String kRadioMaster = "TX16S";
    public static final String kZorro = "Zorro";

    public static final String[] kDriverControllerNames = new String[] {kZorro};
    public static final String[] kOperatorControllerNames = new String[] {kXbox};

    public static final int kDefaultDriverControllerPort = 0;
    public static final int kDefaultOperatorControllerPort = 1;

    public static final class DpadDirection {
      public static int kUp = 0;
      public static int kRight = 90;
      public static int kDown = 180;
      public static int kLeft = 270;
    }
  }

  public static final class AutoConstants {
    public static final int kAllianceColorSelectorPort = 3;

    // max length is 8
    public static final int[] kAutonomousModeSelectorPorts = {0, 1, 2};

    // public static final double kMaxSpeedMetersPerSecond = 3.0;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    // public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    // public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final class TranslationControllerGains {
      public static final double kP = 4.0;
    }

    public static final class RotationControllerGains {
      public static final double kP = 7.0;
    }

    // Constraint for the motion profilied robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    //     new TrapezoidProfile.Constraints(
    //         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ClimberConstants {
    public static final int kClimberPort = 17;
    public static final int kRatchetServoPort = 1;
    public static final double kEngagedPosition = 0 / 1024.0;
    public static final double kDisengedPosition = 1024.0 / 1024.0;

    public static final int kCageSensorPort = 6;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    // By default, the encoder in position mode measures rotations at the motor
    // Convert to inches at the winch
    public static final double kWinchDiameter = 1.0; // inches
    public static final double kGearRatio = 20.0;
    public static final double kPositionConversionFactor = (kWinchDiameter * Math.PI) / kGearRatio;

    // By default, the encoder in velocity mode measures RPM at the motor
    // Convert to inches per second at the winch
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final double kMaxVelocityInchesPerSecond = 5.0;

    public static final double kMaxVelocityRPM =
        kMaxVelocityInchesPerSecond / kVelocityConversionFactor;
    public static final double kMaxAccelerationRPMPerSecond = kMaxVelocityRPM; // 100% accel in 1s

    public static final double kDeployPosition = 8.0; // inches
    public static final double kRetractPosition = 2.5; // inches
  }

  public static final class LedConstants {
    public static final int kLedPort = 9;
    public static final int kLedPixelCount = 40;

    public static final int kLEDsPerBlock = 2;
    public static final int kLEDsBetweenBlocks = 1;

    public static final Color algaeColor = Color.kGreen;
    public static final Color coralColor = Color.kCoral;
  }
}
