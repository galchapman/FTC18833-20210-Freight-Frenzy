package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This auto is used to set robot position after restart
 */
@Autonomous(name = "Generic Red", preselectTeleOp = "Red Drive", group = "Auto: red")
public class GenericRedAuto extends BaseAuto {

    protected GenericRedAuto() {
        super(StartingPosition.FarRed);
        driveTrain.setPose(new Pose2d(0, -1.63, Math.toRadians(90)));
    }

    @Override
    public void initialize() {}

    @Override
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
