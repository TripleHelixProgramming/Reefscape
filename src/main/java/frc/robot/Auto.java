package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drivetrain.Drivetrain;

public class Auto {
  private final Drivetrain m_swerve;
  private final AutoFactory m_autoFactory;

  public Auto(Drivetrain drivetrain) {
    this.m_swerve = drivetrain;
    m_autoFactory =
        new AutoFactory(
            m_swerve::getPose, m_swerve::setPose, m_swerve::followTrajectory, false, m_swerve);
  }

  public AutoRoutine exampleRoutine() {
    AutoRoutine exampleRoutine = m_autoFactory.newRoutine("exampleRoutine");

    AutoTrajectory exampleTrajectory = exampleRoutine.trajectory("examplePath");
    AutoTrajectory exampleTrajectory2 = exampleRoutine.trajectory("examplePath2");

    exampleRoutine
        .active()
        .onTrue(Commands.sequence(exampleTrajectory.resetOdometry(), exampleTrajectory.cmd()));

    exampleTrajectory
        .done()
        .onTrue(Commands.sequence(new WaitCommand(0.5), exampleTrajectory2.cmd()));

    return exampleRoutine;
  }
}
