package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DuckRoller.FancyDuckIndexCommand;
import org.firstinspires.ftc.teamcode.commands.SetRobotArmsPosition;
import org.firstinspires.ftc.teamcode.commands.drive.ArcadeDriveCommand;

@Config
@TeleOp(name="Blue Drive", group = "Drive")
public class BlueDrive extends Drive {

    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, -50, 0.5);
        arcadeDriveCommand = new ArcadeDriveCommand(driveTrain, () -> gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> -gamepad1.right_stick_x, Math.toDegrees(90));
        gp2.dpad_up().whenPressed(new SetRobotArmsPosition(armSubsystem, liftSubsystem, 0.395, 1, 70, 1, 0.525));

        gp1.x().whenPressed(() -> driveTrain.setPose(new Pose2d(0, 0, Math.toRadians(90))));

        fancyDuckIndexCommand = new FancyDuckIndexCommand(ducksSubsystem, Constants.DucksConstants.maxPower, Constants.DucksConstants.minPower, Constants.DucksConstants.accelerationSpeed, Constants.DucksConstants.blueSpin);

//        driveTrain.setDefaultCommand(arcadeDriveCommand);

        gp1.y().whenPressed(fancyDuckIndexCommand);

        gp1.a().whileHeld(arcadeDriveCommand);

        telemetry.addData("ducks spins", () -> ducksSubsystem.getCurrentPosition() / Constants.DucksConstants.ticks_per_rotation);
        telemetry.addData("ducks power", ducksSubsystem::getPower);
    }

    @Override
    public void updateFtcDashboardTelemetry(TelemetryPacket packet) {
        packet.put("ducks spins", ducksSubsystem.getCurrentPosition() / Constants.DucksConstants.ticks_per_rotation);
        packet.put("ducks power", ducksSubsystem.getPower());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
