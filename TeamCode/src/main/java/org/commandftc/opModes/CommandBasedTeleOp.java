package org.commandftc.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.commandftc.Gp;
import org.commandftc.RobotUniversal;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 Command based teleop.
 */
public abstract class CommandBasedTeleOp extends OpMode {

    protected Gp gp1;
    protected Gp gp2;
    private double lastTime = 0;
    // Time between loop runs. Make sure it's stays low
    private double dt = 0;

    @Override
    public final void init() {
        RobotUniversal.setOpMode(this);
        RobotUniversal.opModeType = RobotUniversal.OpModeType.TeleOp;

        gp1 = new Gp(gamepad1);
        gp2 = new Gp(gamepad2);
        assign();
    }

    public abstract void assign();

    @Override
    public final void loop() {
        double time = getRuntime();
        dt = time - lastTime;
        lastTime = time;
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    @Override
    public final void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().close();
        end();
    }

    public void end(){}

    public double dt() {
        return dt;
    }
}
