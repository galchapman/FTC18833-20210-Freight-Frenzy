package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DuckRoller.IndexDuckCommand;
import org.firstinspires.ftc.teamcode.commands.drive.ArcadeDriveCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;

@TeleOp(name="Red Drive")
@Config
public class RedDrive extends Drive {
    public static double indexPower = 0.885;
    public static double rotations = -1.2;
    public static double wait = 0.7;
    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, 50, 0.5);
        indexDuckCommand = new IndexDuckCommand(ducksSubsystem,rotations,indexPower).andThen(new WaitCommand(wait));
        arcadeDriveCommand = new ArcadeDriveCommand(driveTrain, () -> gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> -gamepad1.right_stick_y, Math.toDegrees(-90));


        driveTrain.setDefaultCommand(tankDriveCommand);
        gp1.y().whileHeld(indexDuckCommand);

        gp1.a().whileHeld(arcadeDriveCommand);

        telemetry.addData("index", ducksSubsystem::getCurrentPosition);
        telemetry.addData("index rotation", () -> ducksSubsystem.getCurrentPosition() / Constants.DucksConstants.ticks_per_rotation);
    }

    @Override
    public void updateFtcDashboardTelemetry(TelemetryPacket packet) {
        packet.put("heading", Math.toDegrees(driveTrain.getHeading()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
