package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;
import java.util.Optional;

public class ExampleAuto extends AutoMode {

  public ExampleAuto(Drivetrain drivetrain) {
    super(drivetrain);
  }

  AutoRoutine exampleRoutine = super.getAutoFactory().newRoutine("exampleRoutine");

  AutoTrajectory exampleTrajectory = exampleRoutine.trajectory("examplePath");
  AutoTrajectory exampleTrajectory2 = exampleRoutine.trajectory("examplePath2");

  @Override
  public String getName() {
    return "Example";
  }

  @Override
  public Optional<Pose2d> getInitialPose() {
    return exampleTrajectory.getInitialPose();
  }

  @Override
  public AutoRoutine getAutoRoutine() {

    // spotless:off
    exampleRoutine.active().onTrue(
      Commands.sequence(
        exampleTrajectory.resetOdometry(),
        exampleTrajectory.cmd()
      )
    );

    exampleTrajectory.done().onTrue(
      Commands.sequence(
        new WaitCommand(0.5),
        exampleTrajectory2.cmd()
      )
    );
    // spotless:on

    return exampleRoutine;
  }
}
