package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.drive.RoadRunnerThread;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

@Disabled
@Autonomous(name = "Turn test", group = "tests")
public class TurnTest extends CommandBasedAuto {

    DriveTrainSubsystem driveTrain;
    RoadRunnerThread thread;

    @Override
    public void plan() {
        driveTrain = new DriveTrainSubsystem();
        thread = new RoadRunnerThread(driveTrain);

        driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);
    }

    @Override
    public void onStart() {
        thread.schedule();
    }

    @Override
    public Command getAutonomousCommand() {
        return new TurnCommand(driveTrain, Math.toRadians(360));
    }

    @Override
    public void onEnd() {
        thread.end(true);
    }
}
