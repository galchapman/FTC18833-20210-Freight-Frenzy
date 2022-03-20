package org.commandftc.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.commandftc.RobotUniversal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class CommandBasedAuto extends OpMode {
    private Command autonomousCommand;
    /**
     * DON'T OVERRIDE THIS! IT CALLS init_impl() (WHICH YOU SHOULD INSTEAD OVERRIDE)
     * AND DOES SOMETHING ELSE!!!!!
     */
    @Override
    public final void init() {
        RobotUniversal.setOpMode(this);
        RobotUniversal.opModeType = RobotUniversal.OpModeType.Autonomous;
        plan();
    }
    
    public final void init_loop() {
        telemetry.update();
    }

    @Override
    public final void start() {
        onStart();
        autonomousCommand = getAutonomousCommand();
        autonomousCommand.schedule();
    }

    public void onStart() {}

    public abstract void plan();

    public abstract Command getAutonomousCommand();

    @Override
    public final void loop() {
        CommandScheduler.getInstance().run();
        telemetry.update();
        if (autonomousCommand.isFinished())
            requestOpModeStop();
    }

    @Override
    public final void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().close();
        onEnd();
    }

    public void onEnd() {}
}
