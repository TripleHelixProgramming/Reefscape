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
    private static final double gearRatioMotorToMechanism = 3.0 / 15.0;
    private static final Distance sprocketPitchDiameter = Inches.of(1.7567);
    public static final Distance kPositionConversionFactor =
        sprocketPitchDiameter.times(Math.PI).times(gearRatioMotorToMechanism);
    private static final double maxMotorVelocityRadPerSec = RPM.of(5676).in(RadiansPerSecond);
    private static final LinearVelocity maxTheoreticalVelocity =
        InchesPerSecond.of(maxMotorVelocityRadPerSec * kPositionConversionFactor.in(Inches));

    // By default, the encoder in velocity mode measures RPM at the motor
    // Convert to inches per second at the final stage
    public static final LinearVelocity kVelocityConversionFactor =
        kPositionConversionFactor.per(Minutes);

    public static final LinearVelocity kFineVelocity = InchesPerSecond.of(15.0);
    public static final LinearVelocity kRapidVelocity = InchesPerSecond.of(40.0);
    public static final LinearAccelerationUnit inchesPerSecondPerSecond =
        InchesPerSecond.per(Second);
    public static final LinearAcceleration kRapidAcceleration = inchesPerSecondPerSecond.of(1000);

    public static final Distance kAllowableHeightError = Inches.of(0.2);

    public final class LifterController {
      public static final double kS = 0.13; // Found empirically 2/22/2025
      public static final double kG = 0.194; // Found empirically 2/22/2025
      public static final double kV = (12.0 - kS) / maxTheoreticalVelocity.in(MetersPerSecond);

      public static final double kP = 40.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final Constraints kConstraints =
          new Constraints(
              kRapidVelocity.in(MetersPerSecond), kRapidAcceleration.in(MetersPerSecondPerSecond));
      public static final Distance kIzone = Inches.of(2);
    }

    public static enum LifterState {
      Unknown(0.0),
      Min(0.0),
      AlgaeIntakeFloor(1.0),
      EncoderReset(0.36),
      CoralL1(2.0),
      CoralL2(12.8),
      CoralL3(28.0),
      CoralL4(57.25),
      CoralIntake(11.7),
      AlgaeProcessor(12.0),
      AlgaeL2(28),
      AlgaeL3(44.5),
      Max(67.8);

      public Distance height;

      private LifterState(double inches) {
        this.height = Inches.of(inches);
      }
    }
  }

  public static final class CoralRollerConstants {
    public static final int kMotorPort = 23;

    private static final double gearRatio = 5.0;
    private static final double rollerDiameter = 2.0; // inches

    // By default, the encoder in position mode measures rotations at the motor
    // Convert to inches at the wheel
    public static final double kPositionConversionFactor = (rollerDiameter * Math.PI) / gearRatio;

    // By default, the encoder in velocity mode measures RPM at the drive motor
    // Convert to inches per second at the wheel
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final Voltage kIntakeVoltage = Volts.of(5.0);
    public static final Voltage kOuttakeVoltage = kIntakeVoltage.unaryMinus();
  }

  public static final class CoralWristConstants {
    public static final int kMotorPort = 22;

    private static final double gearRatioMotortoEncoder = 1.0 / 5.0;
    private static final double gearRatioEncoderToArm = 24.0 / 42.0;
    private static final AngularVelocity maxMotorVelocity = RPM.of(5000);
    private static final AngularVelocity maxArmVelocityTheoretical =
        maxMotorVelocity.times(gearRatioMotortoEncoder).times(gearRatioEncoderToArm);
    private static final AngularVelocity maxArmVelocityConstraint = DegreesPerSecond.of(90.0);
    private static final AngularAcceleration maxArmAcceleration =
        DegreesPerSecondPerSecond.of(180.0);
    public static final Constraints kConstraints =
        new Constraints(
            maxArmVelocityConstraint.in(RadiansPerSecond),
            maxArmAcceleration.in(RadiansPerSecondPerSecond));

    /*
     * When used as an absolute encoder, the CTRE SRX Mag encoder measures position
     * in rotations at the sensor by default. Convert to radians at the algae wrist.
     */
    public static final Angle kPositionConversionFactor = Rotations.of(gearRatioEncoderToArm);
    public static final Angle kZeroOffset =
        Degrees.of(63).times(Rotations.of(1.0).div(kPositionConversionFactor));
    public static final Angle kCenterOfGravityOffset = Degrees.of(0.0);
    public static final Angle kAllowableError = Degrees.of(3.0);

    /*
     * When used as an absolute encoder, the CTRE SRX Mag encoder measures velocity
     * in rotations per minute at the sensor by default. Convert to radians per
     * second at the algae wrist.
     */
    public static final AngularVelocity kVelocityConversionFactor =
        Rotations.of(gearRatioEncoderToArm).per(Minute);

    public static final double kG = 0.55; // Found empirically 2/22/2025
    public static final double kS = 0.15; // Found empirically 2/22/2025
    public static final double kV = (12.0 - kS) / maxArmVelocityTheoretical.in(RadiansPerSecond);

    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final Angle kIZone = Degrees.of(30.0);

    public static enum CoralWristState {
      Initial(90),
      Unknown(90),
      Min(15),
      Max(135),
      L1(125),
      L2(65),
      L3(65),
      L4(35),
      Intake(125),
      AlgaeMode(90);

      public Angle angle;

      private CoralWristState(double degrees) {
        this.angle = Degrees.of(degrees);
      }
    }
  }

  public static final class AlgaeRollerConstants {
    public static final int kLeaderMotorPort = 15;
    public static final int kFollowerMotorPort = 16;

    private static final double gearRatio = 5.0;
    private static final double rollerDiameter = 4.0; // inches

    // By default, the encoder in position mode measures rotations at the motor
    // Convert to inches at the wheel
    public static final double kPositionConversionFactor = (rollerDiameter * Math.PI) / gearRatio;

    // By default, the encoder in velocity mode measures RPM at the drive motor
    // Convert to inches per second at the wheel
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    public static final Voltage kIntakeVoltage = Volts.of(5.0);
    public static final Voltage kHoldVoltage = Volts.of(1.0);
    public static final Voltage kOuttakeVoltage = kIntakeVoltage.unaryMinus();
  }

  public static final class AlgaeWristConstants {
    public static final int kMotorPort = 14;

    private static final double gearRatioMotortoEncoder =
        (1.0 / 5.0) * (18.0 / 36.0); // 5:1 for motor, 36:18 for belt
    private static final double gearRatioEncoderToArm = 1.0;
    private static final AngularVelocity maxMotorVelocity = RPM.of(5676);
    private static final AngularVelocity maxArmVelocityTheoretical =
        maxMotorVelocity.times(gearRatioMotortoEncoder).times(gearRatioEncoderToArm);
    private static final AngularVelocity maxArmVelocityConstraint = DegreesPerSecond.of(180.0);
    private static final AngularAcceleration maxArmAcceleration =
        DegreesPerSecondPerSecond.of(180.0);
    public static final Constraints kConstraints =
        new Constraints(
            maxArmVelocityConstraint.in(RadiansPerSecond),
            maxArmAcceleration.in(RadiansPerSecondPerSecond));

    /*
     * When used as an absolute encoder, the REV Through Bore encoder measures
     * position
     * in rotations at the sensor by default. Convert to radians at the algae wrist.
     */
    public static final Angle kPositionConversionFactor = Rotations.of(gearRatioEncoderToArm);
    public static final Angle kZeroOffset = Degrees.of(232.0);
    public static final Angle kCenterOfGravityOffset = Degrees.of(0.0);
    public static final Angle kAllowableError = Degrees.of(3.0);

    /*
     * When used as an absolute encoder, the REV Through Bore encoder measures
     * velocity
     * in rotations per minute at the sensor by default. Convert to radians per
     * second at the algae wrist.
     */
    public static final AngularVelocity kVelocityConversionFactor =
        Rotations.of(gearRatioEncoderToArm).per(Minute);

    public static final double kG = -0.25; // Found empirically 2/22/2025
    public static final double kS = 0.15; // Found empirically 2/22/2025
    public static final double kV = (12.0 - kS) / maxArmVelocityTheoretical.in(RadiansPerSecond);

    public static final double kP = 1.5;
    public static final double kI = 1.0;
    public static final double kD = 0.1;
    public static final Angle kIZone = Degrees.of(30.0);

    public static enum AlgaeWristState {
      Initial(80),
      Unknown(90),
      Floor(0),
      Min(-10),
      Max(95),
      Processor(0),
      L2(-5),
      L3(-5),
      Barge(60),
      CoralMode(80);

      public Angle angle;

      private AlgaeWristState(double degrees) {
        this.angle = Degrees.of(degrees);
      }
    }
  }
}
