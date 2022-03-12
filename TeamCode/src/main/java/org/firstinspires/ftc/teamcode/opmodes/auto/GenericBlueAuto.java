package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This auto is used to set robot position after restart
 */
@Disabled
@Autonomous(name = "Set Alliance Blue", preselectTeleOp = "Blue Drive", group = "Auto: blue")
public class GenericBlueAuto extends BaseAuto {

    public GenericBlueAuto() {
        super(StartingPosition.FarBlue);
    }

    @Override
    public void initialize() {
        driveTrain.setPose(new Pose2d(0, 1.63, Math.toRadians(-90)));
    }

    @Override
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
