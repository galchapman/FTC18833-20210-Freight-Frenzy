package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectoryCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrain;
    private final Trajectory trajectory;
    private boolean wasTrajectoryControlled;
    private long startTime;

    public FollowTrajectoryCommand(DriveTrainSubsystem driveTrain, Trajectory trajectory) {
        this.driveTrain = driveTrain;
        this.trajectory = trajectory;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.followTrajectoryAsync(trajectory);
        wasTrajectoryControlled = driveTrain.trajectoryControlled;
        driveTrain.trajectoryControlled = true;

        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return !driveTrain.isBusy() && System.currentTimeMillis() - startTime > 500;
    }

    @Override
    public void end(boolean interrupted) {
        if (!wasTrajectoryControlled) {
            driveTrain.trajectoryControlled = false;
            driveTrain.stop();
        }
    }
}
