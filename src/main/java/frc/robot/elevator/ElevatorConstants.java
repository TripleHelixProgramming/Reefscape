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
import edu.wpi.first.units.measure.Time;
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
    public static final Time kTimeToMaxVelocity = Seconds.of(0.02);
    public static final LinearAcceleration kRapidAcceleration =
        kRapidVelocity.div(kTimeToMaxVelocity);

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

    public static final double kBeltReduction = 42.0 / 24.0;
    public static final double kTotalGearRatio = 5.0 * kBeltReduction;
    // By default, the REV Through Bore encoder in absolute mode measures rotations
    // Convert to degrees
    public static final double kPositionConversionFactor = 360.0 / kBeltReduction;
    public static final double kPositionOffset = 0.0 / kPositionConversionFactor;

    public static final double kP = 0.3;
    private static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(30.0);
    public static final Time kTimeToMaxVelocity = Seconds.of(0.02);
    private static final AngularAcceleration kMaxAcceleration =
        kMaxVelocity.div(kTimeToMaxVelocity);
    public static final Constraints kConstraints =
        new Constraints(
            kMaxVelocity.in(DegreesPerSecond), kMaxAcceleration.in(DegreesPerSecondPerSecond));
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
    // Convert to inches per `second at the wheel
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final Voltage kIntakeVoltage = Volts.of(5.0);
    public static final Voltage kOuttakeVoltage = kIntakeVoltage.unaryMinus();
  }

  public static final class AlgaeWristConstants {
    public static final int kMotorPort = 14;

    public static final double kTotalGearRatio = 5 * (36 / 18); // 5:1 for motor, 36:18 for belt
    // By default, the REV Through Bore encoder in absolute mode measures rotations
    // Convert to degrees
    public static final double kPositionConversionFactor = 360.0;
    public static final double kPositionOffset = 322.0 / kPositionConversionFactor;

    public static final double kP = 0.001;
    public static final double kI = 0.01;
    public static final double kD = 0.001;
    public static final Angle kIZone = Degrees.of(120.0);

    private static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(150.0);
    public static final Time kTimeToMaxVelocity = Seconds.of(0.5);
    public static final AngularAcceleration kMaxAcceleration = kMaxVelocity.div(kTimeToMaxVelocity);
    public static final Constraints kConstraints =
        new Constraints(
            kMaxVelocity.in(DegreesPerSecond), kMaxAcceleration.in(DegreesPerSecondPerSecond));
    public static final Angle kAllowableAngleError = Degrees.of(3.0);

    public static enum AlgaeWristState {
      Unknown(180),
      Floor(80),
      Min(70),
      Max(190),
      Processor(90),
      L2(80),
      L3(80),
      Barge(140),
      CoralMode(180);

      public Angle angle;

      private AlgaeWristState(double degrees) {
        this.angle = Degrees.of(degrees);
      }
    }
  }
}
