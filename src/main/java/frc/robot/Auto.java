package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceSelector;
import frc.robot.Constants.AutoConstants;
import frc.robot.drivetrain.Drivetrain;

public class Auto {
    private final AllianceSelector m_allianceSelector;
    private final Drivetrain m_swerve;

    private final AutoFactory m_autoFactory;

    public Auto() {
        m_allianceSelector = new AllianceSelector(AutoConstants.kAllianceColorSelectorPort);

        m_swerve = new Drivetrain(m_allianceSelector::fieldRotated);

        m_autoFactory = new AutoFactory(m_swerve::getPose, m_swerve::setPose, m_swerve::followTrajectory, false, m_swerve);
    }

    public class RedTraj {}

    public class BlueTraj {
        Command examplePath = m_autoFactory.trajectoryCmd("examplePath");
    }

    public AutoRoutine exampleRoutine() {
        AutoRoutine exampleRoutine = m_autoFactory.newRoutine("exampleRoutine");

        AutoTrajectory exampleTrajectory = exampleRoutine.trajectory("examplePath");

        exampleRoutine.active().onTrue(
            Commands.sequence(
                exampleTrajectory.resetOdometry(),
                exampleTrajectory.cmd()
            )
        );

        return exampleRoutine;
    }
}
