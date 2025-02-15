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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AlgaeIntakeConstants.AlgaeIntakeStates;
import frc.robot.Constants.CoralIntakeConstants.CoralIntakeStates;
import frc.robot.LEDs.LEDs;
import frc.robot.auto.BlueL4AlgaeAuto;
import frc.robot.auto.ExampleAuto;
import frc.robot.auto.RedL4AlgaeAuto;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DriveToPoseCommand;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.elevator.Elevator;
import frc.robot.grippers.AlgaeIntake;
import frc.robot.grippers.CoralIntake;
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
  private final Drivetrain swerve = new Drivetrain(allianceSelector::fieldRotated);
  private final Climber climber = new Climber();
  private final LEDs leds = new LEDs();
  private final Vision vision = new Vision();
  private final Elevator elevator = new Elevator();
  private final CoralIntake coralIntake = new CoralIntake();
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
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

    climber.setDefaultCommand(climber.createDefaultClimberCommand());

    elevator.setDefaultCommand(
        elevator.createJoystickControlCommand(operator.getHID()));

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
            autoSelector::getSwitchPosition,
            allianceSelector::getAllianceColor,
            allianceSelector::agreementInAllianceInputs));
    leds.setDefaultCommand(
        leds.createDisabledCommand(
            autoSelector::getSwitchPosition,
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
    elevator.resetPositionController();
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
        .onTrue(coralIntake.createSetIntakeVelocityCommand(-5)
        .alongWith(algaeIntake.createSetIntakeVelocityCommand(-5)));

  }

  private void configureOperatorButtonBindings() {

    // Set elevator and grippers to L4
    operator.y().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.L4),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L4.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle)));

    // Set elevator and grippers to L1
    operator.a().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.L1),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L1.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle)));

    // Set elevator and grippers to L2
    operator.b().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.L2),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L2.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle)));

    // Set elevator and grippers to L3
    operator.x().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.L3),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.L3.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle)));

    // Set elevator and grippers to intake position
    operator.start().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.Intake),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.Intake.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.CoralMode.angle)));

    // Set elevator and grippers to barge position
    operator.povUp().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.Max),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Barge.angle)));

    // Set elevator and grippers to L3 algae position
    operator.povRight().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.AlgaeL3),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.L3.angle)));

    // Set elevator and grippers to L2 algae position
    operator.povLeft().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.AlgaeL2),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.L2.angle))); 

    // Set elevator and grippers to processor position
    operator.povDown().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.Processor),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Processor.angle)));

    // Set elevator and grippers to floor intake position
    operator.back().onTrue(
        new ParallelCommandGroup(
            elevator.createSetPositionCommand(ElevatorState.Floor),
            coralIntake.createSetRotationPositionCommand(CoralIntakeStates.AlgaeMode.angle),
            algaeIntake.createSetRotationPositionCommand(AlgaeIntakeStates.Floor.angle)));

    // Intake with coral gripper
    operator.rightBumper()
        .whileTrue(coralIntake.createSetIntakeVelocityCommand(5)
        .onlyIf(() -> elevator.getTargetState() == ElevatorState.Intake));
    
    // Intake with algae gripper
    operator.rightBumper()
        .whileTrue(algaeIntake.createSetIntakeVelocityCommand(5)
        .onlyIf(() -> elevator.getTargetState() != ElevatorState.Intake));

    Trigger elevatorTriggerHigh = operator.axisGreaterThan(Axis.kLeftY.value, 0.9, loop).debounce(0.1);
    Trigger elevatorTriggerLow = operator.axisGreaterThan(Axis.kLeftY.value, -0.9, loop).debounce(0.1);
    elevatorTriggerHigh.or(elevatorTriggerLow).onTrue(elevator.createJoystickControlCommand(operator.getHID()));

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
    autoSelector.addAuto(
        new AutoOption(
            Alliance.Red, 1, new RedL4AlgaeAuto(swerve, elevator, coralIntake, algaeIntake)));
    autoSelector.addAuto(
        new AutoOption(
            Alliance.Blue, 1, new BlueL4AlgaeAuto(swerve, elevator, coralIntake, algaeIntake)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 2, new ExampleAuto(swerve)));
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
