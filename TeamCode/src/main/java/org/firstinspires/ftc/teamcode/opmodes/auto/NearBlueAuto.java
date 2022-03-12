package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.lib.StartingPosition;

@Disabled
@Autonomous(name = "Near Blue", preselectTeleOp = "Blue Drive", group = "Auto: blue")
public class NearBlueAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public NearBlueAuto() {
        super("/sdcard/FIRST/auto/near_blue.auto",
                "/sdcard/FIRST/trajectories/near_blue.json",
                new Pose2d(-1.19, 1.63, Math.toRadians(-90)),
                StartingPosition.NearBlue);
    }
}
