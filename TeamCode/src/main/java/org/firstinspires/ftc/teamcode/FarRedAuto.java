package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@Autonomous(name = "0. Far Red")
public class FarRedAuto extends BaseAuto {

    private static final Pose2d STARTING_POSITION = new Pose2d(-1, -1.61, Math.toRadians(90));

    @Override
    public void initialize() {
        driveTrain.setPoseEstimate(STARTING_POSITION);


        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("heading", () -> Math.toDegrees(driveTrain.getHeading()));
        telemetry.addData("isBusy", driveTrain::isBusy);
    }

    @Override
    public Command getAutonomousCommand() {
        Trajectory trajectory1 = driveTrain.trajectoryBuilder(STARTING_POSITION)
                .splineTo(new Vector2d(-1.45, -0.80), Math.toRadians(90))
//                .forward(0.3)
                .build();

        Trajectory trajectory2 = driveTrain.trajectoryBuilder(trajectory1.end(), true)
                .back(0.5)
                .build();

        return new SequentialCommandGroup(
            follow(trajectory1)
//            follow(trajectory2)
        );
    }
}
