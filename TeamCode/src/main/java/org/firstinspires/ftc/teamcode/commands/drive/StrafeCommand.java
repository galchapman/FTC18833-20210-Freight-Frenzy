package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StrafeCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final double m_distance;

    public StrafeCommand(DriveTrainSubsystem driveTrain, double distance) {
        this.m_driveTrain = driveTrain;
        this.m_distance = distance;

        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {
        Trajectory trajectory = m_driveTrain.trajectoryBuilder(m_driveTrain.getPoseEstimate()).strafeLeft(m_distance).build();
        m_driveTrain.followTrajectoryAsync(trajectory);
    }

    @Override
    public boolean isFinished() {
        return !m_driveTrain.isBusy();
    }
}
