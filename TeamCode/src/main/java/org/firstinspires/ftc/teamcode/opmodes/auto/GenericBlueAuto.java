package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This auto is used to set robot position after restart
 */
@Autonomous(name = "Generic Blue", preselectTeleOp = "Blue Drive", group = "Auto: blue")
public class GenericBlueAuto extends BaseAuto {

    protected GenericBlueAuto() {
        super(StartingPosition.FarBlue);
    }

    @Override
    public void initialize() {}

    @Override
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
