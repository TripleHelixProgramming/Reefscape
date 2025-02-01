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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase{
    private final SparkFlex motor = new SparkFlex(ClimberConstants.kClimberPort, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private final Servo servo = new Servo(ClimberConstants.kRatchetServoPort);

    private final DigitalInput cageSensor = new DigitalInput(ClimberConstants.kCageSensorPort);

    public Climber() {
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
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition());
        SmartDashboard.putBoolean("Cage Sensor", cageSensor.get());
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    private void unlockRatchet() {
        servo.set(ClimberConstants.kDisengedPosition);
    }

    public void lockRatchet() {
        servo.set(ClimberConstants.kEngagedPosition);
    }

    private void setPosition(double targetPosition) {
        controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    }

    private void setVelocity(double targetVelocity) {
        controller.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl);
    }

    // public Command createSetPositionCommand(double position) {
    //     return new InstantCommand(() -> setPosition(position));
    // }

    /**
     * @param controller
     * @param factor Speed scaling factor
     * @return Command that moves the climber arm using the controller joystick
     */
    public Command createClimbByControllerCommand(XboxController controller, double factor) {
        return this.run(() -> this.setVelocity(controller.getRightY() * factor));
    }

    private double getPosition() {
        return encoder.getPosition();
    }

    private Boolean isDeployed() {
        return (Math.abs(encoder.getPosition() - ClimberConstants.kDeployPosition) < 0.1);
    }

    /**
     * @return Command that unlocks and deploys the climber arm
     */
    public Command createDeployCommand() {
        return new FunctionalCommand(
            // initialize
            () -> {
                unlockRatchet();
                setPosition(ClimberConstants.kDeployPosition);
            },
            // execute
            () -> {},
            // end
            interrupted -> lockRatchet(),
            // isFinished
            () -> isDeployed(),
            // requirements
            this);
    }
}
