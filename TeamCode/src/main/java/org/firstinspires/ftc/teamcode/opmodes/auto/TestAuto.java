package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

@Autonomous(name = "Test Auto", group = "Auto: test")
public class TestAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public TestAuto() {
        super("/sdcard/FIRST/auto/test.auto",
                "/sdcard/FIRST/trajectories/test.json",
                new Pose2d(0, 0, Math.toRadians(90)),
                StartingPosition.FarRed);
    }
}
