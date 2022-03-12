package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

@Disabled
@Autonomous(name = "Test Auto", group = "Auto: test")
public class TestAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public TestAuto() {
        super("/sdcard/FIRST/auto/test.auto",
                "/sdcard/FIRST/trajectories/test.json",
                new Pose2d(0.03, -1.63, Math.toRadians(90)),
                StartingPosition.FarRed);
    }
}
