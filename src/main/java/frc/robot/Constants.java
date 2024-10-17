// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Constants {

  public static final class RobotConstants {
    public static final double kNominalVoltage = 12.0;
    public static final double kPeriod = TimedRobot.kDefaultPeriod;
  }

  public static final class DriveConstants {

    public static final class MotorControllers {
      public static final int kRearRightDriveMotorPort = 18;
      public static final int kFrontRightDriveMotorPort = 20;
      public static final int kFrontLeftDriveMotorPort = 28;
      public static final int kRearLeftDriveMotorPort = 10;

      public static final int kRearRightTurningMotorPort = 19;
      public static final int kFrontRightTurningMotorPort = 21;
      public static final int kFrontLeftTurningMotorPort = 29;
      public static final int kRearLeftTurningMotorPort = 11;
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
    public static final Distance kTrackWidth = Inches.of(24);

    // Distance between front and rear wheels on robot
    public static final Distance kWheelBase = Inches.of(22.5);

    // Robot radius
    public static final double kRadius = 0.423;

    public static final LinearVelocity kMaxTranslationalVelocity =
        MetersPerSecond.of(4.0); // max 4.5
    public static final AngularVelocity kMaxRotationalVelocity =
        RadiansPerSecond.of(5.0); // max 5.0

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
  }

  public static final class ModuleConstants {

    public static final int kDriveMotorCurrentLimit = 80;
    public static final int kTurningMotorCurrentLimit = 80;

    public static final double kDriveP = 0.1; // 2023 Competition Robot
    public static final double kDriveI = 0.0; // 2023 Competition Robot
    public static final double kDriveD = 0.0; // 2023 Competition Robot
    public static final double kDriveFF = 0.255; // 2023 Competition Robot

    public static final double kTurningP = 10.0; // 1.5;
    public static final double kTurningI = 0.0; // 2023 Competition Robot
    public static final double kTurningD = 0.0; // 2023 Competition Robot

    // Not adjusted
    // public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0.05 * Math.PI;
    // public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared =
    //    0.05 * Math.PI;

    // public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254,
    // 0.137);

    public static final double kWheelDiameterMeters = 0.10; // 3.7 in; 2023 Competion Robot

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

    public static final class Zorro {
      // RadioMaster Zorro joystick axis
      public static int kLeftXAxis = 0;
      public static int kLeftYAxis = 1;
      public static int kLeftDial = 2;
      public static int kRightDial = 3;
      public static int kRightXAxis = 4;
      public static int kRightYAxis = 5;

      // RadioMaster Zorro buttons
      public static int kBDown = 1;
      public static int kBMid = 2;
      public static int kBUp = 3;
      public static int kEDown = 4;
      public static int kEUp = 5;
      public static int kAIn = 6;
      public static int kGIn = 7;
      public static int kCDown = 8;
      public static int kCMid = 9;
      public static int kCUp = 10;
      public static int kFDown = 11;
      public static int kFUp = 12;
      public static int kDIn = 13;
      public static int kHIn = 14;
    }

    public static final class XBox {
      // XBox Controller D-Pad Constants
      public static int kUp = 0;
      public static int kRight = 90;
      public static int kDown = 180;
      public static int kLeft = 270;
    }
  }

  public static final class AutoConstants {
    public static final int kAllianceColorSelectorPort = 10;

    // length is 8
    public static final int[] kAutonomousModeSelectorPorts = {11, 12, 13, 18, 19};

    // public static final double kMaxSpeedMetersPerSecond = 3.0;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    // public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    // public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants kTranslationControllerGains = new PIDConstants(1.0, 0.0, 0.0);
    public static final PIDConstants kRotationControllerGains = new PIDConstants(7.0, 0.0, 0.0);

    // Constraint for the motion profilied robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    //     new TrapezoidProfile.Constraints(
    //         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
