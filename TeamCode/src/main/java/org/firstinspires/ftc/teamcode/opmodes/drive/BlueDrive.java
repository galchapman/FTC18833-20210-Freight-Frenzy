package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DuckRoller.FancyDuckIndexCommand;
import org.firstinspires.ftc.teamcode.commands.drive.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FieldCentricArcadeDriveCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;

@Config
@TeleOp(name="Blue Drive", group = "Drive")
public class BlueDrive extends Drive {

    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, -50, 0.5);
        GoToSippingHubCommand.setTarget(0.395, 70, 0.57);
        arcadeDriveCommand = new FieldCentricArcadeDriveCommand(driveTrain, () -> gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> -gamepad1.right_stick_x, Math.toDegrees(90));

        gp1.x().whenPressed(() -> driveTrain.setPose(new Pose2d(0, 0, Math.toRadians(90))));

        fancyDuckIndexCommand = new FancyDuckIndexCommand(ducksSubsystem, Constants.DucksConstants.maxPower, Constants.DucksConstants.minPower, Constants.DucksConstants.accelerationSpeed, Constants.DucksConstants.blueSpin);

//        driveTrain.setDefaultCommand(arcadeDriveCommand);

        gp1.y().whenPressed(fancyDuckIndexCommand);

        new Trigger(() -> gamepad1.a && gamepad1.right_bumper).whileActiveContinuous((new ArcadeDriveCommand(driveTrain,() -> 0.15,
                () -> (gp1.left_stick_y() > 0.1) ? 1 :
                        (gp1.left_stick_y() < -0.1 ? -1 : 0), () -> 0, this::getDriveSpeed)));

//        new Trigger(() -> gamepad1.a && gamepad1.right_bumper).whenActive(new AlignRobotCommand(driveTrain, () -> gamepad1.left_stick_y, 0.15, Math.toRadians(90)));
//        new Trigger(() -> gamepad1.a && gamepad1.left_trigger > 0).whenActive(new AlignRobotCommand(driveTrain, () -> gamepad1.left_stick_x, -0.15, 0));

        new Trigger(() -> gamepad1.a && !gamepad1.right_bumper).whileActiveContinuous(arcadeDriveCommand);

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
