package org.commandftc.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.commandftc.Gp;
import org.commandftc.RobotUniversal;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class CommandBasedTeleOp extends OpMode {

    protected Gp gp1;
    protected Gp gp2;
    private double lastTime = 0;
    private double dt = 0;

    /**
     * DON'T OVERRIDE THIS! IT CALLS init_impl() (WHICH YOU SHOULD INSTEAD OVERRIDE)
     * AND DOES SOMETHING ELSE!!!!!
     */
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
