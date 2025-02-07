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

public class RedL4AlgaeAuto extends AutoMode {

  Elevator elevator;

  public RedL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorsubsystem) {
    super(drivetrain);
    elevator = elevatorsubsystem;
  }

  AutoRoutine redL4AlgAutoRoutine = super.getAutoFactory().newRoutine("redL4AlgaeRoutine");

  AutoTrajectory redCenterToL4G = redL4AlgAutoRoutine.trajectory("centerToL4G");
  AutoTrajectory redL4GToAlgae = redL4AlgAutoRoutine.trajectory("L4GToAlgae");
  AutoTrajectory redAlgaeToProcess = redL4AlgAutoRoutine.trajectory("AlgaeToProcess");
  AutoTrajectory redProcessToSource = redL4AlgAutoRoutine.trajectory("ProcessToSource");

  @Override
  public String getName() {
    return "redL4AlgaeAuto";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return redCenterToL4G.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    redL4AlgAutoRoutine
        .active()
        .onTrue(
            Commands.parallel(
                redCenterToL4G.cmd(), elevator.createSetPositionCommand(ElevatorPosition.L4)));

    redCenterToL4G
        .done()
        // .onTrue(m_intake.createOuttakeComamand)
        .onTrue(redL4GToAlgae.cmd());

    redL4GToAlgae
        .done()
        .onTrue(
            Commands.sequence(
                elevator.createSetPositionCommand(
                    ElevatorPosition.L3), /* m_algaeIntake.createIntakeCommand, */
                new WaitCommand(0.2),
                redAlgaeToProcess.cmd()));

    redAlgaeToProcess
        .done()
        // .onTrue(algaeIntake.createOuttakeCommand);
        .onTrue(redProcessToSource.cmd());

    return redL4AlgAutoRoutine;
  }
}
