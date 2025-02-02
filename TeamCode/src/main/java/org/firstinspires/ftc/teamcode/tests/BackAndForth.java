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

@Config
@Disabled
@TeleOp(group = "tests")
public class BackAndForth extends CommandBasedTeleOp {

    public static double DISTANCE = 2;

    DriveTrainSubsystem driveTrain;

    Trajectory forwardTrajectory;
    Trajectory backwardTrajectory;

    double min = Double.MAX_VALUE;
    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();
        driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);

        forwardTrajectory = driveTrain.trajectoryBuilder(this.driveTrain.getPoseEstimate())
                .forward(DISTANCE).build();

        backwardTrajectory = driveTrain.trajectoryBuilder(forwardTrajectory.end())
                .back(DISTANCE).build();

        driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y));

        gp1.x.whenHeld(new FollowTrajectoryCommand(driveTrain, forwardTrajectory));//.andThen(new FollowTrajectoryCommand(driveTrain, backwardTrajectory)));

        telemetry.addData("isBusy", driveTrain::isBusy);
        telemetry.addData("dt", this::dt);

        RobotUniversal.telemetryPacketUpdater = (packet) -> {
            packet.put("lf", driveTrain.getFrontLeftPower());
            if (driveTrain.getFrontLeftPower() < min) {
                min = driveTrain.getFrontLeftPower();
            }
            packet.put("min", min);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        };
    }
}