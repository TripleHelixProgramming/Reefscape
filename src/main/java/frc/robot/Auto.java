package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;

public class Auto {
  private final Drivetrain m_swerve;
  private final Elevator m_elevator;
  private final AutoFactory m_autoFactory;

  public Auto(Drivetrain drivetrain, Elevator elevator) {
    this.m_swerve = drivetrain;
    this.m_elevator = elevator;
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

  public AutoRoutine l4AlgaeToProcess() {
    AutoRoutine l4AlgaeToProcessRoutine = m_autoFactory.newRoutine("l4AlgaeToProcessRoutine");

    AutoTrajectory centerToScore = l4AlgaeToProcessRoutine.trajectory("centerToScore");
    AutoTrajectory score1ToAlgae = l4AlgaeToProcessRoutine.trajectory("score1ToAlgae");
    AutoTrajectory algaeToProcess = l4AlgaeToProcessRoutine.trajectory("algaeToProcess");

    l4AlgaeToProcessRoutine
        .active()
        .onTrue(Commands.parallel(centerToScore.cmd(), m_elevator.createSetPositionCommand(ElevatorPosition.L4)));

    centerToScore
        .done()
        // .onTrue(m_intake.createOuttakeComamand)
        .onTrue(score1ToAlgae.cmd());

    score1ToAlgae
        .done()
        .onTrue(Commands.sequence(m_elevator.createSetPositionCommand(ElevatorPosition.L3), /* m_algaeIntake.createIntakeCommand, */ new WaitCommand(0.2), algaeToProcess.cmd()));

    /* algaeToProcess
        .done()
        .onTrue(m_algaeIntake.createOuttakeCommand);
    */

    return l4AlgaeToProcessRoutine;
  }
}
