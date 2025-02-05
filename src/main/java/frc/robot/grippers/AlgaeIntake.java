package frc.robot.grippers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.RobotConstants;

public class AlgaeIntake extends SubsystemBase{
    
    private final SparkMax intakeLeaderMotor = new SparkMax(AlgaeIntakeConstants.kIntakeLeaderMotorPort, MotorType.kBrushless);
    private final SparkMax intakeFollowerMotor = new SparkMax(AlgaeIntakeConstants.kIntakeFollowerMotorPort, MotorType.kBrushless);
    private final SparkMax rotationMotor = new SparkMax(AlgaeIntakeConstants.kRotationMotorPort, MotorType.kBrushless);

    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();
    private final SparkMaxConfig rotationConfig = new SparkMaxConfig();

    private final RelativeEncoder intakeEncoder;
    private final RelativeEncoder rotationEncoder;

    private final SparkClosedLoopController intakeController;
    private final SparkClosedLoopController rotationController;

    private final DigitalInput coralSensor = new DigitalInput(AlgaeIntakeConstants.kAlgaeSensorPort);

    public AlgaeIntake() {
        intakeConfig
            .voltageCompensation(RobotConstants.kNominalVoltage)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(AlgaeIntakeConstants.kCurrentLimit)
            .inverted(false);

        rotationConfig
            .voltageCompensation(RobotConstants.kNominalVoltage)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(AlgaeIntakeConstants.kCurrentLimit)
            .inverted(false);

        intakeConfig.closedLoop
            .p(AlgaeIntakeConstants.kVelocityP)
            .i(AlgaeIntakeConstants.kVelocityI)
            .d(AlgaeIntakeConstants.kVelocityD)
            .outputRange(-1, 1);

        intakeConfig.encoder
            .velocityConversionFactor(AlgaeIntakeConstants.kVelocityConversionFactor)
            .positionConversionFactor(AlgaeIntakeConstants.kPositionConversionFactor);

        rotationConfig.closedLoop
            .p(AlgaeIntakeConstants.kPositionP)
            .i(AlgaeIntakeConstants.kPositionI)
            .d(AlgaeIntakeConstants.kPositionD)
            .outputRange(-1, 1);

        rotationConfig.encoder
            .velocityConversionFactor(AlgaeIntakeConstants.kVelocityConversionFactor)
            .positionConversionFactor(AlgaeIntakeConstants.kPositionConversionFactor);

        intakeFollowerConfig
            .apply(intakeConfig)
            .follow(intakeLeaderMotor);

        intakeLeaderMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        intakeEncoder = intakeLeaderMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        intakeController = intakeLeaderMotor.getClosedLoopController();
        rotationController = rotationMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotation Position", rotationEncoder.getPosition());
        SmartDashboard.putNumber("Intake Velocity", intakeEncoder.getVelocity());

        SmartDashboard.putBoolean("Coral Sensor", coralSensor.get());
    }

    private void resetIntakeEncoder() {
        intakeEncoder.setPosition(0.0);
    }

    private void resetRotationEncoder() {
        rotationEncoder.setPosition(0.0);
    }

    private void getRotationPosition() {
        rotationEncoder.getPosition();
    }

    private void setRotationPosition(double position) {
        rotationController.setReference(position, ControlType.kPosition);
    }

    private void setIntakeVelocity(double velocity) {
        intakeController.setReference(velocity, ControlType.kVelocity);
    }

    public Trigger hasCoral = new Trigger(() -> coralSensor.get());

    public Command createStopIntakeCommand() {
        return this.runOnce(() -> setIntakeVelocity(0));
    }

    public Command createSetRotationPositionCommand(double position) {
        return this.runOnce(() -> setRotationPosition(position));
    }

    public Command createSetIntakeVelocityCommand(double velocity) {
        return this.run(() -> setIntakeVelocity(velocity));
    }
}
