package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReturnToStartCommand extends CommandBase {
    private final DriveTrainSubsystem driveTrainSubsystem;
    private final Pose2d start;

    public ReturnToStartCommand(DriveTrainSubsystem driveTrainSubsystem, Pose2d start) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.start = start;

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.followTrajectoryAsync(
                driveTrainSubsystem.trajectoryBuilder(driveTrainSubsystem.getPoseEstimate())
                .splineTo(start.vec(), start.getHeading())
                .build()
        );
    }

    @Override
    public boolean isFinished() {
        return !driveTrainSubsystem.isBusy();
    }
}
