package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "0. TestAuto")
public class TestAuto extends LoadedAuto {
    @SuppressLint("SdCardPath")
    public TestAuto() {
        super("/sdcard/FIRST/auto/test.auto");
    }
}