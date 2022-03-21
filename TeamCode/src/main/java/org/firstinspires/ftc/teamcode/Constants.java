package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Rect;

public final class Constants {
    public static final class MotorConstants {
        public static final class RevHDHexMotor {
            public static final int ticks_per_revolution = 28;
            public static final int freeSpeedRPM = 6000;
            public static final int revolutions_per_second = freeSpeedRPM / 60;
        }

        public static final class REVThroughBoreEncoder {
            public static final int ticks_per_revolution = 8192;
        }
    }

    @Config
    public static final class DriveTrainConstants {
        public final static double ticks_per_revolution =
                MotorConstants.REVThroughBoreEncoder.ticks_per_revolution;

        public final static double WheelDiameter = 0.096;
        public final static double HorizontalOdometryWheelDiameter = 0.06;

        public final static double GearRatio = 19.2;

        public final static double TrackWidth = 0.259;
        public final static double MaxVelocity = 1.463453; // Its recommended to put max velocity to 90% of the true value
        public static double MaxAccel = 1;
        public final static double MaxAnglerVelocity = Math.toRadians(90);

        public final static double odometry_wheel_ticks_to_meters = HorizontalOdometryWheelDiameter * Math.PI / ticks_per_revolution;

        public final static double kV = 0.5903;
        public final static double kStatic = 0.12282;
        public static double kA = 0;

        @Config
        public final static class OdometryConstants {
            public static Pose2d frontLeftWheelPosition = new Pose2d(0.16357, -0.07);
            public static Pose2d frontRightWheelPosition = new Pose2d(0.16357, 0.07);

            public static Pose2d horizontalWheelPosition = new Pose2d(-0.15152, 0, Math.toRadians(-90));
        }
    }

    public static final class LiftConstants {
        public static final double inverse_motor_gear = 5.23 * 3.61 * 3.61;
        public static final double ticks_per_motor_revolution = MotorConstants.RevHDHexMotor.ticks_per_revolution * inverse_motor_gear;
        public static final double gear = 15f / 10f;
        public static final int ticks_per_revolution = (int)(ticks_per_motor_revolution / gear);
        public static final double gear_radios = 0.0205;
        public static final double distance_per_revolution = 2 * Math.PI * gear_radios;
        public static final double distance_per_tick = distance_per_revolution / ticks_per_revolution;

        public static final double lower_plate_height = 0.165;
        public static final double top_height = 0.41;
    }

    public static final class ArmConstants {
        public static final int gear = 70;
        public static final int motorGear = MotorConstants.RevHDHexMotor.ticks_per_revolution;
    }

    public static final class IntakeConstants {
        public static final double intakeThreshold = 0.075;
    }

    public static final class DucksConstants {
        public static final double motorGear = 3.61 * 2.89;
        public static final double biggerWheelDiameter = motorGear * 0.0254;
        public static final double wheelDiameter = 0.059;
        public static final double ticks_per_rotation =
                MotorConstants.RevHDHexMotor.ticks_per_revolution * 12
                        * biggerWheelDiameter / wheelDiameter;
        public static final double maxPower = 0.75;
        public static final double minPower = 0.3;
        public static final double accelerationSpeed = 1.5;
        public static final double blueSpin = -1.7;
        public static final double redSpin = 1.7;
    }

    @Config
    public static final class VisionConstants {
        public final static int camera_width = 1280;
        public final static int camera_height = 800;

        public static Rect FarRedARect = new Rect(700, 550, 225, 200);
        public static Rect FarRedBRect = new Rect(1150, 550, 130, 200);
        public static Rect NearRedARect = new Rect(700, 550, 225, 200);
        public static Rect NearRedBRect = new Rect(1150, 550, 130, 200);
        public static Rect FarBlueBRect = new Rect(0, 550, 130, 200);
        public static Rect FarBlueCRect = new Rect(355, 550, 225, 200);
        public static Rect NearBlueBRect = new Rect(0, 550, 130, 200);
        public static Rect NearBlueCRect = new Rect(355, 550, 225, 200);

        public static double HueThresholdLow = 55;
        public static double HueThresholdHigh = 95;
        public static double SaturationThresholdLow = 77;
        public static double SaturationThresholdHigh = 255;
        public static double ValueThresholdLow = 60;
        public static double ValueThresholdHigh = 255;
    }

    public static double getMotorVelocityF(double MaxTicksPerSecond) {
        return 32767 / MaxTicksPerSecond;
    }
}
