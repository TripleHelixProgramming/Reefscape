package frc.robot.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;


public class Elevator extends SubsystemBase{
    
private final SparkMax m_Rmotor;

private final MAXMotionConfig m_MaxMotionConfig = new MAXMotionConfig();

private final SparkMax m_Lmotor;

private final SparkMaxConfig m_defaultMotorConfig = new SparkMaxConfig();
  

  private final RelativeEncoder m_driveEncoder;
  private final EncoderConfig m_driveEncoderConfig = new EncoderConfig();

  private final SparkClosedLoopController m_driveController;
  private final ClosedLoopConfig m_driveControllerConfig = new ClosedLoopConfig();


/**private final ProfiledPIDController m_positionController =
    new ProfiledPIDController(
        IntakeConstants.kPositionP,
        IntakeConstants.kPositionI,
        IntakeConstants.kPositionD,
        IntakeConstants.kConstraints);
        */

        private Elevator() {
            m_Rmotor = new SparkMax(ElevatorConstants.kRmotorport, MotorType.kBrushless);
            m_Lmotor = new SparkMax(ElevatorConstants.kLmotorport, MotorType.kBrushless);

            m_defaultMotorConfig.voltageCompensation(RobotConstants.kNominalVoltage);
            m_defaultMotorConfig.inverted(false);
            m_defaultMotorConfig.idleMode(IdleMode.kBrake);
            m_defaultMotorConfig.smartCurrentLimit(ElevatorConstants.kelevatorcurrantlimit);

            m_driveControllerConfig.p(ElevatorConstants.kP);
            m_driveControllerConfig.i(ElevatorConstants.kI);
            m_driveControllerConfig.d(ElevatorConstants.kD);
            // m_driveControllerConfig.iZone();
            //m_driveControllerConfig.velocityFF(ElevatorConstants.kFF);
            // m_driveControllerConfig.outputRange();
            m_defaultMotorConfig.apply(m_driveControllerConfig);

            m_driveEncoderConfig.positionConversionFactor(ElevatorConstants.kDrivePositionConversionFactor);
            m_driveEncoderConfig.velocityConversionFactor(ElevatorConstants.kDriveVelocityConversionFactor);
            m_defaultMotorConfig.apply(m_driveEncoderConfig);

            m_Rmotor.configure(
              m_defaultMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            m_Lmotor.configure(
              m_defaultMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

              m_driveController = m_Rmotor.getClosedLoopController();
              m_driveEncoder = m_Rmotor.getEncoder();
              //for lead motor (right is lead)

              m_driveController.setConstraints(m_MaxMotionConfig.apply(MAXMotionConfig.));
              m_driveController.setGoal(targetPosition);
              m_driveController.setReference(m_driveEncoder.getPosition(),ControlType.kPosition);


        }

         public Command createStopIntakeCommand() {
    return this.runOnce(
        () -> {
          m_Rmotor.set(0.0);
          m_Lmotor.set(0.0);
        });
}

 
}