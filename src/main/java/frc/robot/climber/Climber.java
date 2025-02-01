package frc.robot.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase{
    private final SparkFlex motor;

    private final SparkFlexConfig motorConfig;

    private final Servo servo;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private final DigitalInput climberSensor;

    public Climber() {
        motor = new SparkFlex(ClimberConstants.kClimberPort, MotorType.kBrushless);

        motorConfig = new SparkFlexConfig();

        servo = new Servo(ClimberConstants.kClimberServoPort);

        motorConfig
            .voltageCompensation(RobotConstants.kNominalVoltage)
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ClimberConstants.kClimberCurrentLimit);

        motorConfig.closedLoop.maxMotion
            .maxAcceleration(ClimberConstants.kMaxAcceleration)
            .maxVelocity(ClimberConstants.kMaxVelocity);

        motorConfig.closedLoop
            .p(ClimberConstants.kP)
            .i(ClimberConstants.kI)
            .d(ClimberConstants.kD)
            .outputRange(-1, 1);

        motorConfig.encoder
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        climberSensor = new DigitalInput(ClimberConstants.kInputSwitchPort);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());

        SmartDashboard.putBoolean("Climber Sensor", climberSensor.get());

        lockOrUnlockRatchet();
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }

    private void ratchetUnlock() {
        servo.set(1);
    }

    private void ratchetLock() {
        servo.set(0);
    }

    private void setPosition(double position) {
        encoder.setPosition(position);
    }

    private void setVelocity(double targetVelocity) {
        controller.setReference(targetVelocity, ControlType.kVelocity);
    }

    private void lockOrUnlockRatchet() {
        if (encoder.getVelocity() > 0) {ratchetUnlock();}
        else {ratchetLock();}
    }

    public Command createSetPositionCommand(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    public Command createClimbByControllerCommand(XboxController controller, double factor) {
        return this.run(() -> this.setVelocity(controller.getRightY() * factor));
    }
}
