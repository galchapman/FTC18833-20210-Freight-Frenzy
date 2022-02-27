package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Disabled
@TeleOp
public class BackAndForth extends CommandBasedTeleOp {

    public static double DISTANCE = 2;

    DriveTrainSubsystem driveTrain;

    Trajectory forwardTrajectory;
    Trajectory backwardTrajectory;

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();

        forwardTrajectory = driveTrain.trajectoryBuilder(this.driveTrain.getPoseEstimate())
                .forward(DISTANCE).build();

        backwardTrajectory = driveTrain.trajectoryBuilder(forwardTrajectory.end())
                .back(DISTANCE).build();

        RobotUniversal.telemetryPacketUpdater = (packet) -> {
            packet.put("xVelocity", driveTrain.getLeftVelocity());
            packet.put("accel", driveTrain.accel);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        };

        driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y));

        gp1.x().whenPressed(new FollowTrajectoryCommand(driveTrain, forwardTrajectory));
    }
}