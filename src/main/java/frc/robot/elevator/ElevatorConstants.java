package frc.robot.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {

  public static final class LifterConstants {
    public static final int kLeaderMotorPort = 24;
    public static final int kFollowerMotorPort = 25;
    public static final int lowerLimitSwitchPort = 9;

    // By default, the encoder in position mode measures rotations at the motor
    // Convert to inches at the final stage
    public static final double kGearRatio = 15.0 / 3.0;
    public static final double kSprocketPitchDiameter = 1.7567; // inches
    public static final double kPositionConversionFactor =
        (kSprocketPitchDiameter * Math.PI) / kGearRatio;

    // By default, the encoder in velocity mode measures RPM at the motor
    // Convert to inches per second at the final stage
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final LinearVelocity kFineVelocity = InchesPerSecond.of(15.0);
    public static final LinearVelocity kRapidVelocity = InchesPerSecond.of(50.0);
    public static final LinearAccelerationUnit inchesPerSecondPerSecond =
        InchesPerSecond.per(Second);
    public static final LinearAcceleration kRapidAcceleration = inchesPerSecondPerSecond.of(1000);

    public static final Distance kAllowableHeightError = Inches.of(0.2);

    public final class LifterController {
      public static final double kP = 0.1;
      public static final Constraints kConstraints =
          new Constraints(
              kRapidVelocity.in(InchesPerSecond), kRapidAcceleration.in(inchesPerSecondPerSecond));
    }

    public static enum LifterState {
      Unknown(0.0),
      Min(0.0),
      Floor(0.0),
      Reset(0.36),
      L1(18.0),
      L2(21.0),
      L3(24.0),
      L4(27.0),
      Intake(12.0),
      Processor(12.0),
      AlgaeL2(30),
      AlgaeL3(46.5),
      Max(30.00);

      public Distance height;

      private LifterState(double inches) {
        this.height = Inches.of(inches);
      }
    }
  }

  public static final class CoralRollerConstants {
    public static final int kMotorPort = 23;
    public static final int kCoralSensorPort = 3;

    public static final double kGearRatio = 5.0;
    public static final double kRollerDiameter = 2.0; // inches

    // By default, the encoder in position mode measures rotations at the motor
    // Convert to inches at the wheel
    public static final double kPositionConversionFactor = (kRollerDiameter * Math.PI) / kGearRatio;

    // By default, the encoder in velocity mode measures RPM at the drive motor
    // Convert to inches per second at the wheel
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final Voltage kIntakeVoltage = Volts.of(5.0);
    public static final Voltage kOuttakeVoltage = kIntakeVoltage.unaryMinus();
  }

  public static final class CoralWristConstants {
    public static final int kMotorPort = 22;

    public static final double kGearRatioMotortoEncoder = 1.0 / 5.0;
    public static final double kGearRatioEncoderToArm = 24.0 / 42.0;

    /*
    When used as an absolute encoder, the CTRE SRX Mag encoder measures position
    in rotations at the sensor by default. Convert to radians at the algae wrist.
     */
    public static final Angle kPositionConversionFactor = Rotations.of(kGearRatioEncoderToArm);
    public static final Angle kPositionOffset =
        Radians.of(0.888).times(Rotations.of(1.0).div(kPositionConversionFactor));

    /*
    When used as an absolute encoder, the CTRE SRX Mag encoder measures velocity
    in rotations per minute at the sensor by default. Convert to radians per second at the algae wrist.
     */
    public static final AngularVelocity kVelocityConversionFactor =
        Rotations.of(kGearRatioEncoderToArm).per(Minute);

    public static final double kP = 4.0;
    public static final double kI = 3.0;
    public static final double kD = 0.03;
    public static final double kG = 2.0;
    public static final Angle kIZone = Degrees.of(30.0);

    private static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
    private static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(180.0);
    public static final Constraints kConstraints =
        new Constraints(
            kMaxVelocity.in(RadiansPerSecond), kMaxAcceleration.in(RadiansPerSecondPerSecond));
    public static final Angle kAllowableAngleError = Degrees.of(3.0);

    public static enum CoralWristState {
      Unknown(125),
      Min(15),
      Max(135),
      L1(90),
      L2(55),
      L3(55),
      L4(25),
      Intake(125),
      AlgaeMode(125);

      public Angle angle;

      private CoralWristState(double degrees) {
        this.angle = Degrees.of(degrees);
      }
    }
  }

  public static final class AlgaeRollerConstants {
    public static final int kLeaderMotorPort = 15;
    public static final int kFollowerMotorPort = 16;
    public static final int kSensorPort = 4;

    public static final double kGearRatio = 5.0;
    public static final double kRollerDiameter = 4.0; // inches

    // By default, the encoder in position mode measures rotations at the motor
    // Convert to inches at the wheel
    public static final double kPositionConversionFactor = (kRollerDiameter * Math.PI) / kGearRatio;

    // By default, the encoder in velocity mode measures RPM at the drive motor
    // Convert to inches per second at the wheel
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final Voltage kIntakeVoltage = Volts.of(5.0);
    public static final Voltage kOuttakeVoltage = kIntakeVoltage.unaryMinus();
  }

  public static final class AlgaeWristConstants {
    public static final int kMotorPort = 14;

    public static final double kGearRatioMotortoEncoder =
        (1.0 / 5.0) * (18.0 / 36.0); // 5:1 for motor, 36:18 for belt
    public static final double kGearRatioEncoderToArm = 1.0;

    /*
    When used as an absolute encoder, the REV Through Bore encoder measures position
    in rotations at the sensor by default. Convert to radians at the algae wrist.
     */
    public static final Angle kPositionConversionFactor = Rotations.of(kGearRatioEncoderToArm);
    public static final Angle kPositionOffset = Degrees.of(232.0);

    /*
    When used as an absolute encoder, the REV Through Bore encoder measures velocity
    in rotations per minute at the sensor by default. Convert to radians per second at the algae wrist.
     */
    public static final AngularVelocity kVelocityConversionFactor =
        Rotations.of(kGearRatioEncoderToArm).per(Minute);

    public static final double kP = 1.0;
    public static final double kI = 1.0;
    public static final double kD = 0.1;
    public static final double kG = 0.0;
    public static final Angle kIZone = Degrees.of(30.0);

    private static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(180.0);
    public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(180.0);
    public static final Constraints kConstraints =
        new Constraints(
            kMaxVelocity.in(RadiansPerSecond), kMaxAcceleration.in(RadiansPerSecondPerSecond));
    public static final Angle kAllowableAngleError = Degrees.of(3.0);

    public static enum AlgaeWristState {
      Unknown(90),
      Floor(-10),
      Min(-20),
      Max(95),
      Processor(0),
      L2(-10),
      L3(-10),
      Barge(80),
      CoralMode(90);

      public Angle angle;

      private AlgaeWristState(double degrees) {
        this.angle = Degrees.of(degrees);
      }
    }
  }
}
