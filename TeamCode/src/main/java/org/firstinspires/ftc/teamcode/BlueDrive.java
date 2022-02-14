package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Blue Drive")
public class BlueDrive extends Drive {
    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, -50, 0.5);
    }
}
