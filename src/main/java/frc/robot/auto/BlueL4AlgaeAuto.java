package frc.robot.auto;

import java.util.Optional;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.elevator.Elevator;

public class BlueL4AlgaeAuto extends AutoMode{

    Elevator elevator;

    public BlueL4AlgaeAuto(Drivetrain drivetrain, Elevator elevatorsubsystem) {
        super(drivetrain);
        elevator = elevatorsubsystem;
    }

    AutoRoutine blueL4AlgAutoRoutine = super.getAutoFactory().newRoutine("BlueL4AlgaeRoutine");

    AutoTrajectory centerToL4 = blueL4AlgAutoRoutine.trajectory("centerToL4");
    AutoTrajectory l4ToAlgae = blueL4AlgAutoRoutine.trajectory("L4ToAlgae");
    AutoTrajectory algaeToProcess = blueL4AlgAutoRoutine.trajectory("AlgaeToProcess");
    AutoTrajectory processToSource = blueL4AlgAutoRoutine.trajectory("ProcessToSource");

    @Override
    public String getName() {
        return "BlueL4AlgaeAuto";
    }

    @Override
    public Optional<Pose2d> getInitialPose() {
        return centerToL4.getInitialPose();
    }

    @Override
    public AutoRoutine getAutoRoutine() {

        blueL4AlgAutoRoutine
        .active()
        .onTrue(Commands.parallel(centerToL4.cmd(), elevator.createSetPositionCommand(ElevatorPosition.L4)));
    
    centerToL4
        .done()
        // .onTrue(m_intake.createOuttakeComamand)
        .onTrue(l4ToAlgae.cmd());
    
    l4ToAlgae
        .done()
        .onTrue(Commands.sequence(elevator.createSetPositionCommand(ElevatorPosition.L3), /* m_algaeIntake.createIntakeCommand, */ new WaitCommand(0.2), algaeToProcess.cmd()));
    
    algaeToProcess
        .done()
        //.onTrue(algaeIntake.createOuttakeCommand);
        .onTrue(processToSource.cmd());

    return blueL4AlgAutoRoutine;
    }
}
