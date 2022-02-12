package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "5. TestAuto")
public class TestAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public TestAuto() {
        super("/sdcard/FIRST/auto/test.auto",
                "/sdcard/FIRST/trajectories/test.json",
                new Pose2d(0.03, -1.63, Math.toRadians(90)));
    }
}