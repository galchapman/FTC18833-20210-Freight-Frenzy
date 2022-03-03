package org.firstinspires.ftc.teamcode.tests.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.LinearOpModeWithCommands;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(group = "tests")
public class MaxAngularVeloTuner extends LinearOpModeWithCommands {
    public static double RUNTIME = 4.0;

    private double maxAngVelocity = 0.0;

    private DriveTrainSubsystem drive;

    @Override
    public void init_subsystems() {
        drive = new DriveTrainSubsystem();
        RobotUniversal.telemetryPacketUpdater = (ignored) -> {};
    }

    @Override
    public void runOpMode() {
        drive = new DriveTrainSubsystem();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);

            telemetry.addData("heading velocity", drive.getExternalHeadingVelocity());
            telemetry.update();
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetry.update();

        while (!isStopRequested()) idle();
    }
}

