package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DuckRoller.FancyDuckIndexCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;

@Config
@TeleOp(name="Blue Drive")
public class BlueDrive extends Drive {
    public static double power0 = 0.5;
    public static double power1 = 0.885;
    public static double time0 = 0.2;
    public static double time1 = 0.7;
    public static double rotations = 1.3;

    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, -50, 0.5);

        indexDuckCommand = new FancyDuckIndexCommand(ducksSubsystem, -rotations, power0, power1, time0)
                .andThen(new WaitCommand(time1));

        gp1.y().whileHeld(indexDuckCommand, false);
    }
}
