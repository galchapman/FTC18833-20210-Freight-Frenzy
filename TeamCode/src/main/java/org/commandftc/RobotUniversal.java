package org.commandftc;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.StartingPosition;

/**
 * This is where the hwMap of the robot is stored.
 */
public final class RobotUniversal {
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;
    public static OpMode opMode;
    public static OpModeType opModeType;
    public static StartingPosition startingPosition;
    public static Consumer<TelemetryPacket> telemetryPacketUpdater = null;

    public enum OpModeType {
        Autonomous,
        TeleOp
    }

    public static void setOpMode(@NonNull OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        RobotUniversal.opMode = opMode;
    }
}
