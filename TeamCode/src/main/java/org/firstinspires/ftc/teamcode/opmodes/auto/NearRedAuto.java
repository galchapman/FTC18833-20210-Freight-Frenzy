package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

@Disabled
@Autonomous(name = "Near Red", preselectTeleOp = "Red Drive", group = "Auto: red")
public class NearRedAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public NearRedAuto() {
        super("/sdcard/FIRST/auto/near_red.auto",
                "/sdcard/FIRST/trajectories/near_red.json",
                new Pose2d(-1.19, -1.63, Math.toRadians(90)),
                StartingPosition.NearRed);
    }
}
