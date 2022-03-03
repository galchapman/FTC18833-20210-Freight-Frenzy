package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectorySequenceCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final TrajectorySequence m_trajectorySequence;
    private boolean wasTrajectoryControlled;

    public FollowTrajectorySequenceCommand(DriveTrainSubsystem driveTrain, TrajectorySequence trajectorySequence) {
        m_driveTrain = driveTrain;
        m_trajectorySequence = trajectorySequence;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        m_driveTrain.followTrajectorySequenceAsync(m_trajectorySequence);
        wasTrajectoryControlled = m_driveTrain.trajectoryControlled;
        m_driveTrain.trajectoryControlled = true;
    }

    @Override
    public void end(boolean interrupted) {
        if (!wasTrajectoryControlled) {
            m_driveTrain.trajectoryControlled = false;
            m_driveTrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return !m_driveTrain.isBusy();
    }
}
