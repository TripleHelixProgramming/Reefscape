package frc.robot.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import java.util.function.BooleanSupplier;

public class Arm extends SubsystemBase {
  private ArmState m_armState;
  private final DoubleSolenoid m_armDeployer;
  private final DoubleSolenoid m_armHardStopper;
  private final DoubleSolenoid m_flapRetracter;

  public Arm() {
    m_armDeployer =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kDeployerForwardChannel,
            ArmConstants.kDeployerReverseChannel);
    m_armHardStopper =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kHardStopperForwardChannel,
            ArmConstants.kHardStopperReverseChannel);
    m_flapRetracter =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kFlapForwardChannel,
            ArmConstants.kFlapReverseChannel);
  }

  @Override
  public void periodic() {
    if (m_armState != null) SmartDashboard.putString("Arm State", m_armState.name());
  }

  public BooleanSupplier stateChecker(ArmState state) {
    return () -> {
      if (this.m_armState != null) return this.m_armState.equals(state);
      else return false;
    };
  }

  private void setState(ArmState state) {
    this.m_armState = state;
  }

  public Command createStowCommand() {
    return this.runOnce(
        () -> {
          this.setState(ArmState.STOWED);
          this.m_armHardStopper.set(Value.kReverse);
          this.m_armDeployer.set(Value.kReverse);
        });
  }

  public Command createDeployCommand() {
    return this.runOnce(
        () -> {
          this.setState(ArmState.DEPLOYED);
          this.m_armHardStopper.set(Value.kReverse);
          this.m_armDeployer.set(Value.kForward);
        });
  }

  public Command createCarryCommand() {
    return this.runOnce(
        () -> {
          this.setState(ArmState.CARRY);
          this.m_armHardStopper.set(Value.kForward);
          this.m_armDeployer.set(Value.kForward);
        });
  }

  public Command createFlapDeployCommand() {
    return this.runOnce(() -> this.m_flapRetracter.set(Value.kForward));
  }

  public Command createFlapRetractCommand() {
    return this.runOnce(() -> this.m_flapRetracter.set(Value.kReverse));
  }
}
