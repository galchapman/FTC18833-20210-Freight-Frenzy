package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.function.DoubleFunction;

public final class Constants {
    public static final class MotorConstants {
        public static final class RevHDHexMotor {
            public static final int ticks_per_revolution = 28;
        }

        public static final class REVThroughBoreEncoder {
            public static final int ticks_per_revolution = 8192;
        }
    }

    public static final class DriveTrainConstants {
        public final static double ticks_per_revolution =
                MotorConstants.REVThroughBoreEncoder.ticks_per_revolution;

        public final static double WheelDiameter = 0.096;
        public final static double HorizontalOdometryWheelDiameter = 0.06;

        public final static double odometry_wheel_ticks_to_meters = HorizontalOdometryWheelDiameter * Math.PI / ticks_per_revolution;

        public final static DoubleFunction<Integer> m_to_ticks = (double m) -> (int)(m / WheelDiameter / Math.PI * ticks_per_revolution);
        public final static DoubleFunction<Double> ticks_to_m = (double ticks) -> ticks * WheelDiameter * Math.PI / ticks_per_revolution;

        public final static double kV = 0.55;
        public final static double kStatic = 0.114;
        public final static double kA = 0.056;

        @Config
        public final static class OdometryConstants {
            public static Pose2d frontLeftWheelPosition = new Pose2d(0.16357, -0.078);
            public static Pose2d frontRightWheelPosition = new Pose2d(0.16357, 0.078);

            public static Pose2d horizontalWheelPosition = new Pose2d(-0.15152, 0, Math.toRadians(-90));
        }
    }

    public static final class LiftConstants {
        public static final int inverse_motor_gear = 80;
        public static final int ticks_per_motor_revolution = MotorConstants.RevHDHexMotor.ticks_per_revolution * inverse_motor_gear;
        public static final double gear = 15f / 10f;
        public static final int ticks_per_revolution = (int)(ticks_per_motor_revolution / gear);
        public static final double gear_radios = 0.0205;
        public static final double distance_per_revolution = 2 * Math.PI * gear_radios;
        public static final double distance_per_tick = distance_per_revolution / ticks_per_revolution;

        public static final double lower_plate_height = 0.165;
        public static final double sensor_height = 0.087;
    }

    public static final class ArmConstants {
        public static final int gear = 70;
        public static final int motorGear = MotorConstants.RevHDHexMotor.ticks_per_revolution;
    }

    public static final class IntakeConstants {
        public static final double intakeThreshold = 0.05;
    }
}
