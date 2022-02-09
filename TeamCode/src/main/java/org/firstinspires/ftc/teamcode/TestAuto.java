package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public TestAuto() {
        super("/sdcard/FIRST/auto/test.auto");
    }
}