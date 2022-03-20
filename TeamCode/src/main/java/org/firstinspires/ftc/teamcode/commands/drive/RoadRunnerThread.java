package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command is used to run a custom thread for our driveTrain.
 */
public class RoadRunnerThread extends CommandBase implements Runnable {
    private final DriveTrainSubsystem m_driveTrain;
    private final Thread m_thread;
    private final AtomicBoolean isRunning = new AtomicBoolean(false);

    public RoadRunnerThread(DriveTrainSubsystem driveTrain) {
        m_driveTrain = driveTrain;
        m_thread = new Thread(this);
    }

    @Override
    public void initialize() {
        if (m_driveTrain.trajectoryControlled) {
            m_driveTrain.isPeriodicThread.set(true);
            isRunning.set(true);
            m_thread.start();
        }
    }

    @Override
    public void execute() {
        isRunning.set(m_driveTrain.trajectoryControlled && isScheduled());
    }

    @Override
    public void run() {
        while (isRunning.get()) {
            m_driveTrain.periodic_impl();
        }
    }

    @Override
    public void end(boolean interrupted) {
        isRunning.set(false);
    }
}
