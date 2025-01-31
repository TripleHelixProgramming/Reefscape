package frc.robot.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase{
    private final SparkMax motor;

    private final SparkMaxConfig motorConfig;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private final DigitalInput climberSensor;

    public Climber() {
        motor = new SparkMax(ClimberConstants.kClimberPort, MotorType.kBrushless);

        motorConfig = new SparkMaxConfig();

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
    }

    private void resetEncoder() {
        encoder.setPosition(0);
    }

    private void setPosition(double position) {
        encoder.setPosition(position);
    }

    private void setVelocity(double targetVelocity) {
        controller.setReference(targetVelocity, ControlType.kVelocity);
    }

    public Command createSetPositionCommand(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    public Command createClimbByControllerCommand(XboxController controller, double factor) {
        return this.run(() -> this.setVelocity(controller.getRightY() * factor));
    }
}
