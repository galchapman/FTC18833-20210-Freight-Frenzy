package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

@Disabled
@Autonomous(name = "Far Red", preselectTeleOp = "Red Drive")
public class FarRedAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public FarRedAuto() {
        super("/sdcard/FIRST/auto/far_red.auto",
                "/sdcard/FIRST/trajectories/far_red.json",
                new Pose2d(0.03, -1.63, Math.toRadians(90)),
                StartingPosition.FarRed);
    }
}