package org.firstinspires.ftc.teamcode;

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

        public final static class OdometryConstants {
            public static final double kMecanum = 2.3384020421638465;

            public static final Pose2d frontLeftWheelPosition = new Pose2d(0.16357, 0.1346);
            public static final Pose2d rearLeftWheelPosition = new Pose2d(-0.16357, 0.1346);
            public static final Pose2d frontRightWheelPosition = new Pose2d(0.16357, -0.1346);
            public static final Pose2d rearRightWheelPosition = new Pose2d(-0.16357, -0.1346);

            public final static Pose2d horizontalWheelPosition = new Pose2d(-0.15152, 0, Math.toRadians(-90));
            public final static Pose2d virtualLeftWheelPosition = new Pose2d(0, 0.1346 * kMecanum);
            public final static Pose2d virtualRightWheelPosition = new Pose2d(0, -0.1346 * kMecanum);
        }
    }

    public static final class LiftConstants {
        public static final int inverse_motor_gear = 4 * 4 * 3;
        public static final int ticks_per_motor_revolution = 10 * inverse_motor_gear;
        public static final double gear = 15f / 10f;
        public static final int ticks_per_revolution = (int)(ticks_per_motor_revolution / gear);
        public static final double gear_radios = 0.0205;
        public static final double distance_per_revolution = 2 * Math.PI * gear_radios;
        public static final double distance_per_tick = distance_per_revolution / ticks_per_revolution;
    }

    public static final class ArmConstants {
        public static final int gear = 70;
        public static final int motorGear = MotorConstants.RevHDHexMotor.ticks_per_revolution;
    }
}
