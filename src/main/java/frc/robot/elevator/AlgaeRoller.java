package frc.robot.elevator;

import static edu.wpi.first.units.Units.FeetPerSecond;

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
import frc.robot.Constants.RobotConstants;
import frc.robot.elevator.ElevatorConstants.AlgaeRollerConstants;

public class AlgaeRoller extends SubsystemBase {

  private final SparkMax leaderMotor =
      new SparkMax(AlgaeRollerConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final SparkMax followerMotor =
      new SparkMax(AlgaeRollerConstants.kFollowerMotorPort, MotorType.kBrushless);

  private final SparkMaxConfig config = new SparkMaxConfig();
  private final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkClosedLoopController controller = leaderMotor.getClosedLoopController();
  private final DigitalInput algaeSensor = new DigitalInput(AlgaeRollerConstants.kSensorPort);

  public AlgaeRoller() {
    // spotless:off
    config
        .voltageCompensation(RobotConstants.kNominalVoltage)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(RobotConstants.kDefaultNEO550CurretnLimit)
        .inverted(false);

    config.closedLoop
        .p(AlgaeRollerConstants.kP)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

    config.encoder
        .velocityConversionFactor(AlgaeRollerConstants.kVelocityConversionFactor)
        .positionConversionFactor(AlgaeRollerConstants.kPositionConversionFactor);

    followerConfig
        .apply(config)
        .follow(leaderMotor);
    // spotless:on

    leaderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    setDefaultCommand(createStopCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Roller Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Algae Roller Applied Duty Cycle", leaderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Algae Roller Current", leaderMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Algae Sensor", algaeSensor.get());
  }

  private void setVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }

  public Trigger hasAlage = new Trigger(() -> algaeSensor.get());

  public Command createStopCommand() {
    return this.startEnd(() -> leaderMotor.set(0.0), () -> {});
  }

  public Command createIntakeCommand() {
    return this.startEnd(
        () -> setVelocity(AlgaeRollerConstants.kIntakeSpeed.in(FeetPerSecond)), () -> {});
  }

  public Command createOuttakeCommand() {
    return this.startEnd(
        () -> setVelocity(AlgaeRollerConstants.kOuttakeSpeed.in(FeetPerSecond)), () -> {});
  }
}
