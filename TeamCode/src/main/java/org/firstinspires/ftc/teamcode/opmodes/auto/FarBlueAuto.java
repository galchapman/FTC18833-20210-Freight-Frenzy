package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

//@Disabled
@Autonomous(name = "Far Blue", preselectTeleOp = "Blue Drive", group = "Auto: blue")
public class FarBlueAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public FarBlueAuto() {
        super("/sdcard/FIRST/auto/far_blue.auto",
                "/sdcard/FIRST/trajectories/far_blue.json",
                new Pose2d(0.03, 1.63, Math.toRadians(-90)),
                StartingPosition.FarBlue);
    }
}
