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

public class AlgaeRoller extends SubsystemBase {

  private final SparkMax rollerLeaderMotor =
      new SparkMax(AlgaeIntakeConstants.kRollerLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax rollerFollowerMotor =
      new SparkMax(AlgaeIntakeConstants.kRollerFollowerMotorPort, MotorType.kBrushless);

  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
  private final SparkMaxConfig rollerFollowerConfig = new SparkMaxConfig();

  private final RelativeEncoder rollerEncoder = rollerLeaderMotor.getEncoder();
  private final SparkClosedLoopController rollerController = rollerLeaderMotor.getClosedLoopController();
  private final DigitalInput algaeSensor = new DigitalInput(AlgaeIntakeConstants.kAlgaeSensorPort);

  public AlgaeRoller() {
    // spotless:off
    rollerConfig
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    rollerConfig.closedLoop
        .p(AlgaeIntakeConstants.kVelocityP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

    rollerConfig.encoder
        .velocityConversionFactor(AlgaeIntakeConstants.kVelocityConversionFactor)
        .positionConversionFactor(AlgaeIntakeConstants.kPositionConversionFactor);

    rollerFollowerConfig
        .apply(rollerConfig)
        .follow(rollerLeaderMotor);
    // spotless:on

    rollerLeaderMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerFollowerMotor.configure(
        rollerFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Intake Velocity", rollerEncoder.getVelocity());
    SmartDashboard.putBoolean("Algae Sensor", algaeSensor.get());
  }

  private void resetIntakeEncoder() {
    rollerEncoder.setPosition(0.0);
  }

  private void setIntakeVelocity(double velocity) {
    rollerController.setReference(velocity, ControlType.kVelocity);
  }

  public Trigger hasCoral = new Trigger(() -> algaeSensor.get());

  public Command createStopCommand() {
    return this.runOnce(() -> setIntakeVelocity(0));
  }

  public Command createSetIntakeVelocityCommand(double velocity) {
    return this.run(() -> setIntakeVelocity(velocity));
  }
}
