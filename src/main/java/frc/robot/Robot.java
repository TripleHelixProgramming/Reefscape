package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerPatroller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LEDs.LEDs;
import frc.robot.auto.BlueL4AlgaeAuto;
import frc.robot.auto.BlueNoProcess3PieceAuto;
import frc.robot.auto.ExampleAuto;
import frc.robot.auto.RedL4AlgaeAuto;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DriveToPoseCommand;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConstants.LifterConstants.LifterState;
import frc.robot.elevator.Lifter;
import frc.robot.vision.Vision;
import java.util.HashMap;
import java.util.Map;

public class Robot extends TimedRobot {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private final AllianceSelector allianceSelector =
      new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);
  private final AutoSelector autoSelector =
      new AutoSelector(
          AutoConstants.kAutonomousModeSelectorPorts, allianceSelector::getAllianceColor);

  // private final Lifter elevator = new Lifter();
  private final Elevator elevator = new Elevator();
  private final Lifter lifter = elevator.getLifter();
  private final Drivetrain swerve =
      new Drivetrain(allianceSelector::fieldRotated, lifter::getProportionOfMaxHeight);
  private final Climber climber = new Climber();
  private final LEDs leds = new LEDs();
  private final Vision vision = new Vision();

  private CommandZorroController driver;
  private CommandXboxController operator;
  private int usbCheckDelay = OIConstants.kUSBCheckNumLoops;
  private Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();
  private final EventLoop loop = new EventLoop();

  private StructArrayPublisher<Pose2d> reefTargetPositionsPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Reef Target Positions", Pose2d.struct)
          .publish();

  public Robot() {
    configureButtonBindings();
    configureEventBindings();
    configureAutoOptions();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));
  }

  @Override
  public void robotInit() {
    // Start recording to data log
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#logging-joystick-data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    swerve.setDefaultCommand(
        new ZorroDriveCommand(swerve, DriveConstants.kDriveKinematics, driver.getHID()));

    reefTargetPositionsPublisher.set(DriveConstants.kReefTargetPoses);
  }

  @Override
  public void robotPeriodic() {
    loop.poll();
    CommandScheduler.getInstance().run();
    checkVision();
    SmartDashboard.putData("Driver Controller", driver.getHID());
    SmartDashboard.putData("Operator Controller", operator.getHID());
    SmartDashboard.putData(powerDistribution);
  }

  @Override
  public void disabledInit() {
    leds.setDefaultCommand(
        leds.createDisabledCommand(
            autoSelector::getBinarySwitchPosition,
            allianceSelector::getAllianceColor,
            allianceSelector::agreementInAllianceInputs));
    leds.setDefaultCommand(
        leds.createDisabledCommand(
            autoSelector::getBinarySwitchPosition,
            allianceSelector::getAllianceColor,
            allianceSelector::agreementInAllianceInputs));
  }

  @Override
  public void disabledPeriodic() {
    // Scan the USB devices. If they change, remap the buttons.

    /*
     * Only check if controllers changed every kUSBCheckNumLoops loops of disablePeriodic().
     * This prevents us from hammering on some routines that cause the RIO to lock up.
     */
    usbCheckDelay--;
    if (0 >= usbCheckDelay) {
      usbCheckDelay = OIConstants.kUSBCheckNumLoops;
      if (ControllerPatroller.getInstance().controllersChanged()) {
        // Reset the joysticks & button mappings.
        configureButtonBindings();
      }
    }

    allianceSelector.disabledPeriodic();
    autoSelector.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    autoSelector.scheduleAuto();
    leds.setDefaultCommand(leds.createEnabledCommand());
    climber.lockRatchet();
    lifter.setDefaultCommand(lifter.createRemainAtCurrentHeightCommand());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    autoSelector.cancelAuto();
    leds.setDefaultCommand(leds.createEnabledCommand());
    swerve.resetHeadingOffset();
    climber.resetEncoder();
    climber.lockRatchet();
    elevator.resetPositionControllers();
    lifter.setDefaultCommand(lifter.createJoystickControlCommand(operator.getHID()));
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
    driver = new CommandZorroController(cp.findDriverPort());
    operator = new CommandXboxController(cp.findOperatorPort());

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    driver.DIn()
        .onTrue(new InstantCommand(() -> swerve.setHeadingOffset())
        .ignoringDisable(true));

    // Drive to nearest pose
    driver.AIn()
        .whileTrue(new DriveToPoseCommand(swerve, vision, () -> swerve.getNearestPose()));

    // Outtake grippers
    driver.HIn()
        .onTrue(elevator.getCoralRoller().createOuttakeCommand()
        .alongWith(elevator.getAlgaeRoller().createOuttakeCommand()));

  }

  private void configureOperatorButtonBindings() {

    Trigger algaeMode = operator.leftBumper();

    // Configure to either score coral on L1 or score algae in processor
    operator.a().whileTrue(new ConditionalCommand(elevator.algaeProcessorPositionCG(), elevator.coralL1PositionCG(), algaeMode));

    // Configure to either score coral on L2 or intake algae from L2
    operator.b().whileTrue(new ConditionalCommand(elevator.algaeL2IntakeCG(), elevator.coralL2PositionCG(), algaeMode));

    // Configure to either score coral on L3 or intake algae from L3
    operator.x().whileTrue(new ConditionalCommand(elevator.algaeL3IntakeCG(), elevator.coralL3PositionCG(), algaeMode));

    // Configure to either score coral on L4 or score algae in barge
    operator.y().whileTrue(new ConditionalCommand(elevator.algaeBargePositionCG(), elevator.coralL4PositionCG(), algaeMode));

    // Configure to either intake coral from source or intake algae from floor
    operator.start().whileTrue(new ConditionalCommand(elevator.algaeFloorIntakeCG(), elevator.coralIntakeCG(), algaeMode));

    // Intake with coral gripper
    operator.rightBumper()
        .whileTrue(elevator.getCoralRoller().createIntakeCommand()
        .onlyIf(() -> lifter.getTargetState() == LifterState.Intake));
    
    // Intake with algae gripper
    operator.rightBumper()
        .whileTrue(elevator.getAlgaeRoller().createIntakeCommand()
        .onlyIf(() -> lifter.getTargetState() != LifterState.Intake));

    // Force joystick operation of the elevator
    Trigger elevatorTriggerHigh = operator.axisGreaterThan(Axis.kLeftY.value, 0.9, loop).debounce(0.1);
    Trigger elevatorTriggerLow = operator.axisGreaterThan(Axis.kLeftY.value, -0.9, loop).debounce(0.1);
    elevatorTriggerHigh.or(elevatorTriggerLow).onTrue(lifter.createJoystickControlCommand(operator.getHID()));

    // Actuate climber winch
    Trigger climbTrigger = operator.axisGreaterThan(Axis.kRightY.value, -0.9, loop).debounce(0.1);
    climbTrigger.onTrue(climber.createDeployCommand()
        .andThen(climber.createClimbByControllerCommand(operator.getHID(), -ClimberConstants.kMaxVelocityInchesPerSecond)));
  }
  // spotless:on

  private void configureEventBindings() {
    autoSelector.getChangedAutoSelection().onTrue(leds.createChangeAutoAnimationCommand());
  }

  private void configureAutoOptions() {
    autoSelector.addAuto(new AutoOption(Alliance.Red, 1, new RedL4AlgaeAuto(swerve, elevator)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 1, new BlueL4AlgaeAuto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Blue, 2, new BlueNoProcess3PieceAuto(swerve, elevator)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 3, new ExampleAuto(swerve)));
  }

  /**
   * Gets the current drawn from the Power Distribution Hub by a CAN motor controller, assuming that
   * (PDH port number + 10) = CAN ID
   *
   * @param CANBusPort The CAN ID of the motor controller
   * @return Current in Amps on the PDH channel corresponding to the motor channel
   */
  public double getPDHCurrent(int CANBusPort) {
    return powerDistribution.getCurrent(CANBusPort - 10);
  }

  private synchronized StructPublisher<Pose2d> getPose2dPublisher(String name) {
    var publisher = posePublishers.get(name);
    if (publisher == null) {
      publisher = NetworkTableInstance.getDefault().getStructTopic(name, Pose2d.struct).publish();
      posePublishers.put(name, publisher);
    }
    return publisher;
  }

  protected void checkVision() {
    vision
        .getPoseEstimates()
        .forEach(
            est -> {
              swerve.addVisionMeasurement(
                  est.pose().estimatedPose.toPose2d(), est.pose().timestampSeconds, est.stdev());
              getPose2dPublisher(est.name()).set(est.pose().estimatedPose.toPose2d());
            });
  }
}
