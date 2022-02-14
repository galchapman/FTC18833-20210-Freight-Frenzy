package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DuckRoller.IndexDuckCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;

@TeleOp(name="Blue Drive")
public class BlueDrive extends Drive {
    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, -50, 0.5);

        indexDuckCommand = new IndexDuckCommand(ducksSubsystem, -1.3, 0.7).andThen(new WaitCommand(2));

        gp1.y().whileHeld(indexDuckCommand);
    }
}
