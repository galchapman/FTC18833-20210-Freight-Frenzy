package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

@Autonomous(name = "Turn test", group = "tests")
public class TurnTest extends CommandBasedAuto {

    DriveTrainSubsystem driveTrain;

    @Override
    public void plan() {
        driveTrain = new DriveTrainSubsystem();

        driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);
    }

    @Override
    public Command getAutonomousCommand() {
        return new TurnCommand(driveTrain, Math.toRadians(360));
    }
}
