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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.game.Gamepiece;
import frc.game.Reef;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerPatroller;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LEDs.LEDs;
import frc.robot.auto.BlueL4Auto;
import frc.robot.auto.BlueMoveAuto;
import frc.robot.auto.BlueNoProcess3PieceAuto;
import frc.robot.auto.BlueProcess3PieceAuto;
import frc.robot.auto.RedL4Auto;
import frc.robot.auto.RedMoveAuto;
import frc.robot.auto.RedNoProcess3PieceAuto;
import frc.robot.auto.RedProcess3PieceAuto;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DriveToPoseCommand;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.elevator.AlgaeRoller;
import frc.robot.elevator.AlgaeWrist;
import frc.robot.elevator.CoralRoller;
import frc.robot.elevator.CoralWrist;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Lifter;
import frc.robot.vision.Vision;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Robot extends TimedRobot {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private final AllianceSelector allianceSelector =
      new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);
  private final AutoSelector autoSelector =
      new AutoSelector(
          AutoConstants.kAutonomousModeSelectorPorts, allianceSelector::getAllianceColor);

  private final Elevator elevator = new Elevator();
  private final Lifter lifter = elevator.getLifter();
  private final CoralRoller coralRoller = elevator.getCoralRoller();
  private final CoralWrist coralWrist = elevator.getCoralWrist();
  private final AlgaeRoller algaeRoller = elevator.getAlgaeRoller();
  private final AlgaeWrist algaeWrist = elevator.getAlgaeWrist();

  private final Drivetrain swerve =
      new Drivetrain(allianceSelector::fieldRotated, lifter::getProportionOfMaxHeight);
  private final Climber climber = new Climber();
  private final LEDs leds = LEDs.getInstance();
  private final Vision vision = new Vision();
  private final EventLoop loop = new EventLoop();

  private CommandZorroController driver;
  private CommandXboxController operator;
  private BooleanSupplier algaeModeSupplier;
  private Supplier<Gamepiece> gamepieceSupplier;
  private int usbCheckDelay = OIConstants.kUSBCheckNumLoops;
  private Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

  private StructArrayPublisher<Pose2d> reefTargetPositionsPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Reef Target Positions", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> leftCoralPipeTargetPositionsPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Sector left pipe target", Pose2d.struct)
          .publish();
  private StructPublisher<Pose2d> rightCoralPipeTargetPositionsPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Sector right pipe target", Pose2d.struct)
          .publish();

  public Robot() {
    gamepieceSupplier =
        new Supplier<Gamepiece>() {
          @Override
          public Gamepiece get() {
            return getLoadedGamepiece();
          }
        };

    configureButtonBindings();
    configureEventBindings();
    configureAutoOptions();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    addPeriodic(() -> swerve.refreshRelativeTurningEncoder(), 0.1);
  }

  @Override
  public void robotInit() {
    // Start recording to data log
    // https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#logging-joystick-data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    reefTargetPositionsPublisher.set(DriveConstants.kReefTargetPoses);
  }

  @Override
  public void robotPeriodic() {
    loop.poll();
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    CommandScheduler.getInstance().run();
    checkVision();
    SmartDashboard.putData("Driver Controller", driver.getHID());
    SmartDashboard.putData("Operator Controller", operator.getHID());
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(swerve);
    SmartDashboard.putData(climber);
    SmartDashboard.putData(leds);
    SmartDashboard.putData(powerDistribution);
    elevator.periodic();
    SmartDashboard.putString(
        "Gamepiece", getLoadedGamepiece() == null ? "None" : getLoadedGamepiece().toString());

    var nearestReef = Reef.getNearestReef(swerve.getPose());
    var nearestReefFace = nearestReef.getNearestFace(swerve.getPose());
    var nearestLeftPipe = nearestReefFace.getLeftPipePose();
    var nearestRightPipe = nearestReefFace.getRightPipePose();
    var nearestRedReefFace = Reef.Red.getNearestFace(swerve.getPose());

    SmartDashboard.putString("Sector nearest reef", nearestReef.toString());
    SmartDashboard.putString("Sector nearest reef face", nearestReefFace.toString());
    SmartDashboard.putString("Sector nearest red reef face", nearestRedReefFace.toString());
    leftCoralPipeTargetPositionsPublisher.set(nearestLeftPipe);
    rightCoralPipeTargetPositionsPublisher.set(nearestRightPipe);
  }

  @Override
  public void disabledInit() {
    leds.replaceDefaultCommandImmediately(
        leds.createAutoOptionDisplayCommand(
                autoSelector,
                () -> swerve.getPose(),
                allianceSelector.getAgreementInAllianceColor())
            .ignoringDisable(true));

    // autoSelector.getChangedAutoSelection().onChange(leds.createAutoSelectionEffectCommand().withTimeout(Seconds.of(3)));
  }

  @Override
  public void disabledPeriodic() {
    // Scan the USB devices. If they change, remap the buttons.

    /*
     * Only check if controllers changed every kUSBCheckNumLoops loops of
     * disablePeriodic().
     * This prevents us from hammering on some routines that cause the RIO to lock
     * up.
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
    swerve.setDefaultCommand(swerve.createStopCommand());
    lifter.setDefaultCommand(lifter.remainAtCurrentHeight());
    leds.replaceDefaultCommandImmediately(
        leds.createStandardDisplayCommand(algaeModeSupplier, gamepieceSupplier));
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    autoSelector.cancelAuto();
    swerve.setDefaultCommand(
        new ZorroDriveCommand(swerve, DriveConstants.kDriveKinematics, driver.getHID()));
    lifter.setDefaultCommand(lifter.joystickVelocityControl(operator.getHID()));
    leds.replaceDefaultCommandImmediately(
        leds.createStandardDisplayCommand(algaeModeSupplier, gamepieceSupplier));

    // Test wrist feedforwards
    // algaeWrist.setDefaultCommand(algaeWrist.createJoystickControlCommand(operator.getHID()));
    // coralWrist.setDefaultCommand(coralWrist.createJoystickControlCommand(operator.getHID()));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  Gamepiece getLoadedGamepiece() {
    if (algaeRoller.hasAlgae.getAsBoolean()) {
      return Gamepiece.ALGAE;
    } else if (coralRoller.hasCoral.getAsBoolean()) {
      return Gamepiece.CORAL;
    }
    return null;
  }

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
    driver.GIn()
        .onTrue(new InstantCommand(() -> {
          swerve.setHeadingOffset();
          // swerve.initializeRelativeTurningEncoder();
        }).ignoringDisable(true));

    // Drive to nearest pose
    // driver.AIn()
    //     .whileTrue(new DriveToPoseCommand(swerve, () -> swerve.getNearestPose()));

    driver.AIn().whileTrue(
        new DriveToPoseCommand(swerve, 
          () -> Reef.getNearestReef(swerve.getPose()).getNearestFace(swerve.getPose()).getLeftPipePose()));
    driver.DIn().whileTrue(
        new DriveToPoseCommand(swerve, 
          () -> Reef.getNearestReef(swerve.getPose()).getNearestFace(swerve.getPose()).getRightPipePose()));

    // Outtake grippers
    var outtaking = driver.HIn();
    lifter.atProcessorHeight.and(outtaking)
        .whileTrue(algaeRoller.createOuttakeToProcessorCommand());
    lifter.atProcessorHeight.negate().and(outtaking)
        .whileTrue(algaeRoller.createOuttakeToBargeCommand());
    outtaking
        .whileTrue(coralRoller.createOuttakeCommand());
  }

  private void configureOperatorButtonBindings() {

    var algaeMode = operator.leftBumper();
    algaeModeSupplier = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return algaeMode.getAsBoolean();
      }
    };

    // Test wrist motion
    // operator.back()
    // .onTrue(coralWrist.createSetAngleCommand(CoralWristState.AlgaeMode)
    // .alongWith(algaeWrist.createSetAngleCommand(AlgaeWristState.Floor)));
    // operator.start()
    // .onTrue(coralWrist.createSetAngleCommand(CoralWristState.L4)
    // .alongWith(algaeWrist.createSetAngleCommand(AlgaeWristState.CoralMode)));

    // Test algae roller motion
    // operator.back().whileTrue(algaeRoller.createIntakeCommand());
    // operator.start().whileTrue(algaeRoller.createOuttakeCommand());

    // Test coral roller motion
    // operator.back().whileTrue(coralRoller.createIntakeCommand());
    // operator.start().whileTrue(coralRoller.createOuttakeCommand());

    // Configure to either score coral on L1 or score algae in processor
    operator.a().onTrue(new ConditionalCommand(
        elevator.algaeProcessorPositionCG(), elevator.coralL1PositionCG(), algaeMode));

    // Configure to either score coral on L2 or intake algae from L2
    operator.b().onTrue(new ConditionalCommand(
        elevator.algaeL2IntakeCG(), elevator.coralL2PositionCG(), algaeMode));

    // Configure to either score coral on L3 or intake algae from L3
    operator.x().onTrue(new ConditionalCommand(
        elevator.algaeL3IntakeCG(), elevator.coralL3PositionCG(), algaeMode));

    // Configure to either score coral on L4 or score algae in barge
    operator.y().onTrue(new ConditionalCommand(
        elevator.algaeBargePositionCG(), elevator.coralL4PositionCG(), algaeMode));

    // Configure to either intake coral from source or intake algae from floor
    // operator.start().whileTrue(new ConditionalCommand(
    //     elevator.algaeFloorIntakeCG(), elevator.coralIntakeCG(), algaeMode));

    operator.rightTrigger().whileTrue(new ConditionalCommand(
        elevator.algaeFloorIntakeCG(), elevator.coralIntakeCG(), algaeMode));

    // Intake coral and algae
    operator.rightBumper()
        .whileTrue(algaeRoller.createIntakeCommand()
        .alongWith(coralRoller.createIntakeCommand()));

    // Force joystick operation of the elevator
    Trigger elevatorTriggerHigh = operator.axisGreaterThan(Axis.kLeftY.value, 0.9, loop).debounce(0.1);
    Trigger elevatorTriggerLow = operator.axisGreaterThan(Axis.kLeftY.value, -0.9, loop).debounce(0.1);
    // elevatorTriggerHigh.or(elevatorTriggerLow).onTrue(lifter.createJoystickControlCommand(operator.getHID()));

    // Actuate climber winch
    // Trigger climbTrigger = operator.axisGreaterThan(Axis.kRightY.value, -0.9, loop).debounce(0.1);
    // climbTrigger.onTrue(climber.createDeployCommand()
    //     .andThen(climber.createClimbByControllerCommand(operator.getHID(), -ClimberConstants.kMaxVelocityInchesPerSecond)));

    operator.leftTrigger().onTrue(climber.createDeployCommand());

    // Auto climb to position
    operator.povUp().onTrue(climber.createRetractCommand());

    /*
     * Left and right D-pad buttons will cause the robot to go to the left/right
     * pipe on the nearest reef face.
     */
    // operator.povLeft().whileTrue(
    //   new DriveToPoseCommand(swerve, 
    //   () -> Reef.getNearestReef(swerve.getPose()).getNearestFace(swerve.getPose()).getLeftPipePose()));
    // operator.povRight().whileTrue(
    //   new DriveToPoseCommand(swerve, 
    //   () -> Reef.getNearestReef(swerve.getPose()).getNearestFace(swerve.getPose()).getRightPipePose()));

    // just for testing roller animation.
    // operator.povLeft().whileTrue(leds.createRollerAnimationCommand(() -> true, () -> Color.kOrange));
    // operator.povRight().whileTrue(leds.createRollerAnimationCommand(() -> false, () -> Color.kOrange));
  }

  private void configureEventBindings() {
    var autonomous = RobotModeTriggers.autonomous();
    autonomous.onTrue(elevator.resetPositionControllers());
    autonomous.onTrue(climber.lockRatchet());
    
    var teleop = RobotModeTriggers.teleop();
    teleop.onTrue(swerve.resetHeadingOffset());
    teleop.onTrue(lifter.matchHeight());
    teleop.onTrue(elevator.resetPositionControllers());
    teleop.onTrue(climber.lockRatchet()
        .andThen(climber.resetEncoder()));

    algaeRoller.hasAlgae
        .whileTrue(algaeRoller.createHoldAlgaeCommand());
    // algaeRoller.hasAlgae
    //     .onTrue(algaeWrist.createSetAngleCommand(AlgaeWristState.Barge));
    coralRoller.isRolling.or(algaeRoller.isRolling).whileTrue(createRollerAnimationCommand());
  }

  private void configureAutoOptions() {
    autoSelector.addAuto(
        new AutoOption(Alliance.Red, 1, new RedL4Auto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Blue, 1, new BlueL4Auto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Red, 2, new RedNoProcess3PieceAuto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Blue, 2, new BlueNoProcess3PieceAuto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Red, 3, new RedProcess3PieceAuto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Blue, 3, new BlueProcess3PieceAuto(swerve, elevator)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Blue, 4, new BlueMoveAuto(swerve)));
    autoSelector.addAuto(
        new AutoOption(Alliance.Red, 4, new RedMoveAuto(swerve)));
  }
  // spotless:on

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

  /**
   * Create a command that animates the rollers based on their current state.
   *
   * <p>If both rollers are running we assume it's an outtake and use the color of the current
   * gamepiece (yellow if none). If only one roller is running, we assume its an intake and use the
   * color of piece associated with that roller.
   *
   * <p>After working out the logic, the command is created by the LED subsystem.
   *
   * @return An LED subsystem command that animates the rollers.
   */
  protected Command createRollerAnimationCommand() {
    System.err.printf(
        "createRollerAnimation: algae=%b, coral=%b%n",
        algaeRoller.isRolling, coralRoller.isRolling);
    /*
     * Both rollers run for intake and outtake now, so need to use motor direction
     * to determine intake vs. outtake
     */
    BooleanSupplier intakeSupplier =
        () -> {
          return coralRoller.getRollerVelocity() > 1 || algaeRoller.getRollerVelocity() > 1;
        };

    /*
     * Use currently held gamepiece to color outtake animation, yellow if none.
     * On intake, use the color associated with the gripper's gamepiece.  On
     * some weird logic error, use red.
     */
    Supplier<Color> colorSupplier =
        () -> {
          /*
           * On outtake, use the color of the currently held game piece, yellow if none.
           */
          if (!intakeSupplier.getAsBoolean()) {
            var gamepiece = getLoadedGamepiece();
            return gamepiece == null ? Color.kYellow : gamepiece.color;
          }
          /*
           * Otherwise, use the color associated with the current control mode.
           */
          else if (algaeModeSupplier.getAsBoolean()) {
            return Gamepiece.ALGAE.color;
          } else {
            return Gamepiece.CORAL.color;
          }
        };

    return leds.createRollerAnimationCommand(intakeSupplier, colorSupplier);
  }
}
