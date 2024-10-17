// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.ControllerPatroller;
import frc.lib.SendableXBoxController;
import frc.lib.SendableZorroController;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.OIConstants.XBox;
import frc.robot.Constants.OIConstants.Zorro;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import java.util.function.IntSupplier;

public class RobotContainer {

  private static RobotContainer INSTANCE = null;

  public static RobotContainer getRobotContainer() {
    if (INSTANCE == null) {
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }

  private final PowerDistribution m_PowerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private final Drivetrain m_swerve = new Drivetrain();

  private final EventLoop m_loop = new EventLoop();
  private SendableZorroController m_driver;
  private SendableXBoxController m_operator;

  // digital inputs for autonomous selection
  private final DigitalInput[] autonomousModes =
      new DigitalInput[AutoConstants.kAutonomousModeSelectorPorts.length];

  private Autonomous m_autonomous;

  public RobotContainer() {

    configureButtonBindings();

    for (int i = 0; i < AutoConstants.kAutonomousModeSelectorPorts.length; i++) {
      autonomousModes[i] = new DigitalInput(AutoConstants.kAutonomousModeSelectorPorts[i]);
    }

    setDefaultCommands();

    m_swerve.configurePathPlanner();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));
  }

  public void configureButtonBindings() {

    // Clear any active buttons.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    ControllerPatroller cp = ControllerPatroller.getInstance();

    // We use two different types of controllers - Joystick & XboxController.
    // Create objects of the specific types.
    m_driver = new SendableZorroController(cp.findDriverPort());
    m_operator = new SendableXBoxController(cp.findOperatorPort());

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  /**
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    updateSelectedAutonomous();
    if (m_autonomous != null) return m_autonomous.getPathPlannerAuto();
    else return null;
  }

  public void teleopInit() {}

  public void autonomousInit() {}

  public void disabledInit() {}

  public void periodic() {
    m_loop.poll();
    updateSelectedAutonomous();

    if (m_autonomous != null) {
      SmartDashboard.putString("Auto", m_autonomous.getFilename());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }

    SmartDashboard.putData(m_driver);
    SmartDashboard.putData(m_operator);
    SmartDashboard.putData(m_PowerDistribution);
  }

  private class Autonomous {

    private final String filename;

    private Autonomous(String filename) {
      this.filename = filename;
    }

    private Command getPathPlannerAuto() {
      return new PathPlannerAuto(filename);
    }

    private String getFilename() {
      return filename;
    }
  }

  // spotless:off
  /** Updates the autonomous based on the physical selector switch */
  private void updateSelectedAutonomous() {
    switch (getAutonomousModeSwitchIndex()) {
      case 1:
        m_autonomous =
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? new Autonomous("R-TheOnePiece")
                : new Autonomous("B-TheOnePiece");
        break;

      case 2:
        m_autonomous =
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? new Autonomous("R-TheTwoPieceNear")
                : new Autonomous("B-TheTwoPieceNear");
        break;

      case 3:
        m_autonomous =
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? new Autonomous("R-TwoPieceFar1")
                : new Autonomous("B-TwoPieceFar1");
        break;

      case 4:
        m_autonomous = 
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? new Autonomous("R-ThreePieceAutoTame") 
                : new Autonomous("B-ThreePieceAutoTame");
        break;

      case 5:
        m_autonomous = 
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? null
                : null;
            break;

      default:
        m_autonomous = null;
    }
  }
  // spotless:on

  /**
   * @return Index in array of Digital Inputs corresponding to selected auto mode
   */
  private int getAutonomousModeSwitchIndex() {
    for (int port = 0; port < autonomousModes.length; port++) {
      if (!autonomousModes[port].get()) {
        return port + 1;
      }
    }
    return 0; // failure of the physical switch
  }

  private IntSupplier autonomousModeSelector() {
    return () -> getAutonomousModeSwitchIndex();
  }

  /**
   * Gets the current drawn from the Power Distribution Hub by a CAN motor controller, assuming that
   * (PDH port number + 10) = CAN ID
   *
   * @param CANBusPort The CAN ID of the motor controller
   * @return Current in Amps on the PDH channel corresponding to the motor channel
   */
  public double getPDHCurrent(int CANBusPort) {
    return m_PowerDistribution.getCurrent(CANBusPort - 10);
  }

  private void setDefaultCommands() {
    m_swerve.setDefaultCommand(
        new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematics, m_driver));
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    new JoystickButton(m_driver, Zorro.kHIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading())
        .ignoringDisable(true));

    new JoystickButton(m_driver, Zorro.kAIn)
    .whileTrue((new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematicsDriveFromArm, m_driver)));

  }
  // spotless:on

  private void configureOperatorButtonBindings() {}

}
