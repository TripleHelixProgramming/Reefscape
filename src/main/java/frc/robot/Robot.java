package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerPatroller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LEDs.LEDs;
import frc.robot.autos.ChoreoAuto;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import java.util.ArrayList;
import java.util.List;

import choreo.auto.AutoFactory;

public class Robot extends TimedRobot {

  private List<AutoOption> m_autoOptions = new ArrayList<>();

  private final PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private final AllianceSelector m_allianceSelector;
  private final AutoSelector m_autoSelector;
  private final Drivetrain m_swerve;
  private final LEDs m_LEDs;

  private final AutoFactory m_autoFactory;

  private CommandZorroController m_driver;
  private CommandXboxController m_operator;

  private int m_usb_check_delay = OIConstants.kUSBCheckNumLoops;

  public Robot() {
    m_allianceSelector = new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);

    configureAutoOptions();
    m_autoSelector =
        new AutoSelector(
            AutoConstants.kAutonomousModeSelectorPorts,
            m_allianceSelector::getAllianceColor,
            m_autoOptions);

    m_swerve = new Drivetrain(m_allianceSelector::fieldRotated);
    m_LEDs = new LEDs();

    configureButtonBindings();
    configureEventBindings();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    m_autoFactory = new AutoFactory(m_swerve::getPose, m_swerve::setPose, m_swerve::followTrajectory, false, m_swerve);
  }

  @Override
  public void robotInit() {
    // Starts recording to data log
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#logging-joystick-data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_swerve.setDefaultCommand(
        new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematics, m_driver.getHID()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(m_driver.getHID());
    SmartDashboard.putData(m_operator.getHID());
    SmartDashboard.putData(m_powerDistribution);
  }

  @Override
  public void disabledInit() {
    m_LEDs.setDefaultCommand(
        m_LEDs.createDisabledCommand(
            m_autoSelector::getSwitchPosition,
            m_allianceSelector::getAllianceColor,
            m_allianceSelector::agreementInAllianceInputs));
  }

  @Override
  public void disabledPeriodic() {
    // Scan the USB devices. If they change, remap the buttons.

    /*
     * Only check if controllers changed every kUSBCheckNumLoops loops of disablePeriodic().
     * This prevents us from hammering on some routines that cause the RIO to lock up.
     */
    m_usb_check_delay--;
    if (0 >= m_usb_check_delay) {
      m_usb_check_delay = OIConstants.kUSBCheckNumLoops;
      if (ControllerPatroller.getInstance().controllersChanged()) {
        // Reset the joysticks & button mappings.
        configureButtonBindings();
      }
    }

    m_allianceSelector.disabledPeriodic();
    m_autoSelector.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    m_autoSelector.scheduleAuto();
    m_LEDs.setDefaultCommand(m_LEDs.createEnabledCommand());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_autoSelector.cancelAuto();
    m_LEDs.setDefaultCommand(m_LEDs.createEnabledCommand());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  public void configureButtonBindings() {

    // Clear any active buttons.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    ControllerPatroller cp = ControllerPatroller.getInstance();

    // We use two different types of controllers - Joystick & XboxController.
    // Create objects of the specific types.
    m_driver = new CommandZorroController(cp.findDriverPort());
    m_operator = new CommandXboxController(cp.findOperatorPort());

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    m_driver.HIn()
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading())
        .ignoringDisable(true));

  }
  // spotless:on

  private void configureOperatorButtonBindings() {}

  private void configureEventBindings() {
    m_autoSelector.getChangedAutoSelection().onTrue(m_LEDs.createChangeAutoAnimationCommand());
  }

  private void configureAutoOptions() {
    m_autoOptions.add(new AutoOption(Alliance.Red, 4));
    m_autoOptions.add(new AutoOption(Alliance.Blue, 1));
  }

  /**
   * Gets the current drawn from the Power Distribution Hub by a CAN motor controller, assuming that
   * (PDH port number + 10) = CAN ID
   *
   * @param CANBusPort The CAN ID of the motor controller
   * @return Current in Amps on the PDH channel corresponding to the motor channel
   */
  public double getPDHCurrent(int CANBusPort) {
    return m_powerDistribution.getCurrent(CANBusPort - 10);
  }

  public ChoreoAuto auto = new ChoreoAuto() {

    final String getName() {
      return "Example Auto";
    }

    public Command getCommand() {
      return Commands.sequence(
        m_autoFactory.resetOdometry("pickupGamepiece"), 
        Commands.deadline(
            m_autoFactory.trajectoryCmd("pickupGamepiece")
            // intakeSubsystem.intake() 
        ),
        Commands.parallel(
            m_autoFactory.trajectoryCmd("scoreGamepiece")
            // scoringSubsystem.getReady()
        )
        // scoringSubsystem.score()
      );
    }
  }
}
