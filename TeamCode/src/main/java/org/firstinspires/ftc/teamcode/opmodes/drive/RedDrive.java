package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DuckRoller.FancyDuckIndexCommand;
import org.firstinspires.ftc.teamcode.commands.SetRobotArmsPosition;
import org.firstinspires.ftc.teamcode.commands.drive.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@TeleOp(name="Red Drive", group = "Drive")
@Config
public class RedDrive extends Drive {
    public static double maxPower = 0.885;
    public static double minPower = 0.55;
    public static double accelerationSpeed = 0.55;
    public static double wait = 0.7;
    @Override
    public void assign() {
        super.assign();
        GoToScoringPositionCommand.setTarget(0.20, 50, 0.5);
        fancyDuckIndexCommand = new FancyDuckIndexCommand(ducksSubsystem, Constants.DucksConstants.maxPower, Constants.DucksConstants.minPower, Constants.DucksConstants.accelerationSpeed, Constants.DucksConstants.redSpin);
        arcadeDriveCommand = new ArcadeDriveCommand(driveTrain, () -> gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> -gamepad1.right_stick_y, Math.toDegrees(-90));

        gp1.x().whenPressed(() -> driveTrain.setPose(new Pose2d(0, 0, Math.toRadians(-90))));

        driveTrain.setDefaultCommand(tankDriveCommand);
        gp1.y().whenPressed(fancyDuckIndexCommand);

        gp1.a().whileHeld(arcadeDriveCommand);

        telemetry.addData("index", ducksSubsystem::getCurrentPosition);
        telemetry.addData("index rotation", () -> ducksSubsystem.getCurrentPosition() / Constants.DucksConstants.ticks_per_rotation);

        Trajectory trajectorySequence = driveTrain.trajectoryBuilder(new Pose2d(0.4, -1.663, 0), true)
                .strafeTo(new Vector2d(-0.275, -1.175))
                .build();

        gp2.dpad_up().whenPressed(new SetRobotArmsPosition(armSubsystem, liftSubsystem, 0.395, 1, -70, 1, 0.575));


        new Trigger(() -> driveTrain.getLineColorSensorBrightness() > 100
                && Math.abs(-Math.PI - (driveTrain.getDriveHeading() + driveTrain.getHeading())) < Math.toRadians(20)).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {driveTrain.setPose(new Pose2d(0.835, -1.663, driveTrain.getPoseEstimate().getHeading()));
                            driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);}),
                        new WaitCommand(0.5),
                        new InstantCommand(() ->
                                    new FollowTrajectoryCommand(driveTrain, trajectorySequence).andThen(
                                            new InstantCommand(
                                                    () -> driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Up))
                                    ).withInterrupt(() -> gamepad1.b).schedule()
                                )
                ).withInterrupt(() -> gamepad1.b)
        );
        new Trigger(() -> driveTrain.getLineColorSensorBrightness() > 100 && Math.abs(-Math.PI - (driveTrain.getDriveHeading() + driveTrain.getHeading())) < Math.toRadians(20) && driveTrain.getRightDistance() < 0.20)
                .whenActive(() -> gamepad1.rumble(500));
    }

    @Override
    public void updateFtcDashboardTelemetry(TelemetryPacket packet) {
        packet.put("heading", Math.toDegrees(driveTrain.getHeading()));
        packet.put("drive heading", Math.toDegrees(driveTrain.getDriveHeading() + driveTrain.getHeading()));
        packet.put("velocity", driveTrain.accel);
        packet.put("intake distance", intakeSubsystem.getDistance());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
