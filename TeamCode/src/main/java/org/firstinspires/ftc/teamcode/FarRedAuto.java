package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@Autonomous(name = "Far Red")
public class FarRedAuto extends BaseAuto {

    @Override
    public void initialize() {
        telemetry.addData("heading", () -> Math.toDegrees(driveTrain.getExternalHeading()));
    }

    @Override
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                turn(360)
        );
    }
}