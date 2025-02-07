package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;
import java.util.Optional;

public class BlueL4AlgaeAuto extends AutoMode {

  Elevator elevator;

  public BlueL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorsubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
  }

  AutoRoutine blueL4AlgAutoRoutine = super.getAutoFactory().newRoutine("BlueL4AlgaeRoutine");

  AutoTrajectory blueCenterToL4G = blueL4AlgAutoRoutine.trajectory("centerToL4G");
  AutoTrajectory blueL4GToAlgae = blueL4AlgAutoRoutine.trajectory("L4GToAlgae");
  AutoTrajectory blueAlgaeToProcess = blueL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  AutoTrajectory blueProcessToSource = blueL4AlgAutoRoutine.trajectory("ProcessToSource");

  @Override
  public String getName() {
    return "BlueL4AlgaeAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return blueCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    blueL4AlgAutoRoutine
        .active()
        .onTrue(
            Commands.parallel(
                blueCenterToL4G.cmd(), elevator.createSetPositionCommand(ElevatorPosition.L4)));

    blueCenterToL4G
        .done()
        // .onTrue(m_intake.createOuttakeComamand)
        .onTrue(blueL4GToAlgae.cmd());

    blueL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevator.createSetPositionCommand(
                    ElevatorPosition.L3), /* m_algaeIntake.createIntakeCommand, */
                new WaitCommand(0.2),
                blueAlgaeToProcess.cmd()));

    blueAlgaeToProcess
        .done()
        // .onTrue(algaeIntake.createOuttakeCommand);
        .onTrue(blueProcessToSource.cmd());

    return blueL4AlgAutoRoutine;
  }
}
