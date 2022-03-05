package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.lib.Encoder;
import org.firstinspires.ftc.teamcode.lib.drive.ArcadeDrive;
import org.firstinspires.ftc.teamcode.lib.drive.HorizontalDrive;
import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequenceRunner;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.commandftc.RobotUniversal.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.MaxAccel;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.MaxAnglerVelocity;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.MaxVelocity;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.TrackWidth;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.kA;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.kStatic;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.kV;
import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.odometry_wheel_ticks_to_meters;

@Config
public class DriveTrainSubsystem extends MecanumDrive implements TankDrive, ArcadeDrive, HorizontalDrive {
    private final DcMotorEx m_FrontLeftMotor;
    private final DcMotorEx m_RearLeftMotor;
    private final DcMotorEx m_FrontRightMotor;
    private final DcMotorEx m_RearRightMotor;

    private final Encoder m_leftOdometryEncoder;
    private final Encoder m_rightOdometryEncoder;
    private final Encoder m_horizontalOdometryEncoder;

    private final Servo m_odometryServo;

    private final Rev2mDistanceSensor m_leftDistanceSensor;
    private final Rev2mDistanceSensor m_rightDistanceSensor;

    private final ColorSensor m_ColorSensor;

    private final BNO055IMU imu;

    public static PIDCoefficients FORWARD_PID = new PIDCoefficients(4, 0, 0); // 4
    public static PIDCoefficients STRAFE_PID = new PIDCoefficients(7, 0, 0); // 7
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0); // 5

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MaxVelocity, MaxAnglerVelocity, TrackWidth);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MaxAccel);

    public boolean trajectoryControlled;

    private double angleOffset;

    // if Periodic running on it's own thread
    public final AtomicBoolean isPeriodicThread = new AtomicBoolean(false);

    public enum OdometryPosition {
        Up(1),
        Down(0.45);

        private final double servo_position;
        OdometryPosition(double pos) {
            servo_position = pos;
        }
    }

    public DriveTrainSubsystem() {
        super(kV, kA, kStatic, TrackWidth, 0);
        // Hardware
        m_FrontLeftMotor  = (DcMotorEx) hardwareMap.dcMotor.get("FrontLeftDriveMotor");
        m_RearLeftMotor   = (DcMotorEx) hardwareMap.dcMotor.get("RearLeftDriveMotor");
        m_FrontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("FrontRightDriveMotor");
        m_RearRightMotor  = (DcMotorEx) hardwareMap.dcMotor.get("RearRightDriveMotor");
        m_leftOdometryEncoder = new Encoder(m_FrontLeftMotor);
        m_rightOdometryEncoder = new Encoder(m_FrontRightMotor);
        m_horizontalOdometryEncoder = new Encoder((DcMotorEx) hardwareMap.dcMotor.get("IntakeMotor"));
        m_odometryServo = hardwareMap.servo.get("OdometryServo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        m_leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "LeftDistanceSensor");
        m_rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "RightDistanceSensor");
        m_ColorSensor = hardwareMap.get(ColorSensor.class, "LineColorSensor");
        // IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        // Initialize drive motors
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Odometry
        setOdometryPosition(OdometryPosition.Up);
        setLocalizer(new TwoTrackingWheelLocalizer(
                Arrays.asList(
                        DriveTrainConstants.OdometryConstants.frontLeftWheelPosition,
                        DriveTrainConstants.OdometryConstants.horizontalWheelPosition
                )) {
            @NonNull
            @Override
            public List<Double> getWheelPositions() {
                return Arrays.asList(getLeftPosition(), getHorizontalPosition());
            }

            @Override
            public double getHeading() {
                return DriveTrainSubsystem.this.getHeading();
            }

            @Override
            public Double getHeadingVelocity() {
                return (double) imu.getAngularVelocity().zRotationRate;
            }

            @Override
            public List<Double> getWheelVelocities() {
                return Arrays.asList(getLeftVelocity(), getHorizontalVelocity());
            }
        });
//        setLocalizer(new ThreeTrackingWheelLocalizer(Arrays.asList(
//                        OdometryConstants.frontLeftWheelPosition,
//                        OdometryConstants.frontRightWheelPosition,
//                        OdometryConstants.horizontalWheelPosition
//        )) {
//            @NonNull
//            @Override
//            public List<Double> getWheelPositions() {
//                return Arrays.asList(getFrontLeftPosition(), getFrontRightPosition(), getHorizontalPosition());
//            }
//        });

        TrajectoryFollower follower = new HolonomicPIDVAFollower(FORWARD_PID, STRAFE_PID, HEADING_PID,
                new Pose2d(0.02, 0.02, Math.toRadians(5)), 0.5);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        trajectoryControlled = RobotUniversal.opModeType == RobotUniversal.OpModeType.Autonomous; // check if Opmode is autonomous

        CommandScheduler.getInstance().registerSubsystem(this); // Because we aren't extending SubsystemBase
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return new ArrayList<>();
    }

    public double accel = 0;
    long lt = 0;
    double lv = 0;


    public void periodic() {
        if (!isPeriodicThread.get()) {
            periodic_impl();
        }
    }

    public void periodic_impl() {
        if (trajectoryControlled) {
            updatePoseEstimate();
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
            if (signal != null)
                setDriveSignal(signal);
        } else {
            updatePoseEstimate();
            trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        }

        // make angle continues
        double angle = getHeading();
        double delta = angle - lastAngle;
        if (delta > Math.PI / 2) {
            spinCount--;
        } else if (delta < -Math.PI / 2) {
            spinCount++;
        }
        lastAngle = angle;

        long time = System.nanoTime();
        double velocity = getLeftVelocity();
        double current_accel = (velocity - lv) / (time - lt) * 1000000000;
        if (Math.abs(current_accel) < 2) {
            accel = current_accel;
        }
        lt = time;
        lv = velocity;
    }

    public void setOdometryPosition(OdometryPosition position) {
        m_odometryServo.setPosition(position.servo_position);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        m_FrontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_RearLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_FrontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_RearRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return m_FrontLeftMotor.getZeroPowerBehavior();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        m_FrontLeftMotor.setMode(mode);
        m_RearLeftMotor.setMode(mode);
        m_FrontRightMotor.setMode(mode);
        m_RearRightMotor.setMode(mode);
    }

    public void setDrivePIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        m_FrontLeftMotor.setPIDFCoefficients(mode, pidfCoefficients);
        m_RearLeftMotor.setPIDFCoefficients(mode, pidfCoefficients);
        m_FrontRightMotor.setPIDFCoefficients(mode, pidfCoefficients);
        m_RearRightMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    public DcMotor.RunMode getDriveMode() {
        return m_FrontLeftMotor.getMode();
    }

    @Override
    public void stop() {
        m_FrontLeftMotor.setPower(0);
        m_RearLeftMotor.setPower(0);
        m_FrontRightMotor.setPower(0);
        m_RearRightMotor.setPower(0);
    }

    public void driveForward(double power) {
        m_FrontLeftMotor.setPower(power);
        m_RearLeftMotor.setPower(power);
        m_FrontRightMotor.setPower(power);
        m_RearRightMotor.setPower(power);
    }

    @Override
    public void tankDrive(double left, double right) {
        m_FrontLeftMotor.setPower(left);
        m_RearLeftMotor.setPower(left);
        m_FrontRightMotor.setPower(right);
        m_RearRightMotor.setPower(right);
    }

    @Override
    public void arcadeDrive(double x, double y,double spin) {
        m_FrontLeftMotor.setPower(x + y + spin);
        m_RearLeftMotor.setPower(-x + y + spin);
        m_FrontRightMotor.setPower(-x + y - spin);
        m_RearRightMotor.setPower(x + y - spin);
    }

    public void setPowers(double frontLeft, double rearLeft, double frontRight, double rearRight) {
        m_FrontLeftMotor.setPower(frontLeft);
        m_RearLeftMotor.setPower(rearLeft);
        m_FrontRightMotor.setPower(frontRight);
        m_RearRightMotor.setPower(rearRight);
    }

    @Override
    public void driveLeft(double power) {
        m_FrontLeftMotor.setPower(-power);
        m_RearLeftMotor.setPower(power);
        m_FrontRightMotor.setPower(power);
        m_RearRightMotor.setPower(-power);
    }

    @Override
    public void driveRight(double power) {
        m_FrontLeftMotor.setPower(power);
        m_RearLeftMotor.setPower(-power);
        m_FrontRightMotor.setPower(-power);
        m_RearRightMotor.setPower(power);
    }

    public int getLeftEncoder() {
        return m_leftOdometryEncoder.getCurrentPosition();
    }

    public int getRightEncoder() {
        return m_rightOdometryEncoder.getCurrentPosition();
    }

    public int getHorizontalEncoder() {
        return m_horizontalOdometryEncoder.getCurrentPosition();
    }

    public double getLeftPosition() {
        return getLeftEncoder() * odometry_wheel_ticks_to_meters;
    }

    public double getRightPosition() {
        return getRightEncoder() * odometry_wheel_ticks_to_meters;
    }

    public double getHorizontalPosition() {
        return getHorizontalEncoder() * odometry_wheel_ticks_to_meters;
    }

    public double getFrontLeftPower() {
        return m_FrontLeftMotor.getPower();
    }

    public double getRearLeftPower() {
        return m_RearLeftMotor.getPower();
    }

    public double getFrontRightPower() {
        return m_FrontRightMotor.getPower();
    }

    public double getRearRightPower() {
        return m_RearRightMotor.getPower();
    }

    public double getLeftVelocity() {
        return m_leftOdometryEncoder.getCorrectedVelocity() * odometry_wheel_ticks_to_meters;
    }

    public double getRightVelocity() {
        return m_rightOdometryEncoder.getCorrectedVelocity() * odometry_wheel_ticks_to_meters;
    }

    public double getHorizontalVelocity() {
        return m_horizontalOdometryEncoder.getCorrectedVelocity() * odometry_wheel_ticks_to_meters;
    }

    public double getHeading() {
        return getRawExternalHeading() + angleOffset;
    }

    public int getLineColorSensorBrightness(){
        return m_ColorSensor.alpha();
    }

    public void setPose(Pose2d pose) {
        setPoseEstimate(pose);
        angleOffset = pose.getHeading() - getRawExternalHeading();
    }

    private double lastAngle = 0;
    private int spinCount = 0;
    public double getContainsHeading() {
        return lastAngle + spinCount * 2 * Math.PI;
    }

    @Override
    protected double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    @Override
    public void setMotorPowers(double fl, double rl, double rr, double fr) {
        setPowers(fl, rl, fr, rr);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, TrajectoryVelocityConstraint VEL_CONSTRAINT) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                Math.toRadians(165), Math.toRadians(165)
        );
    }

    @NotNull
    @Contract("_, _, _ -> new")
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    @NotNull
    @Contract("_ -> new")
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void followTrajectoryAsync(@NotNull Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public boolean areDriveMotorsBusy() {
        return m_FrontLeftMotor.isBusy() || m_RearLeftMotor.isBusy() || m_FrontRightMotor.isBusy() || m_RearRightMotor.isBusy();
    }

    public double getLeftDistance() {
        return m_leftDistanceSensor.getDistance(DistanceUnit.METER);
    }

    public double getRightDistance() {
        return m_rightDistanceSensor.getDistance(DistanceUnit.METER);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = Math.abs(drivePower.getX())
                    + Math.abs(drivePower.getY())
                    + Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    drivePower.getX(),
                    drivePower.getY(),
                    drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public double getDriveHeading() {
        double fl = m_FrontLeftMotor.getPower();
        double rl = m_RearLeftMotor.getPower();
        double fr = m_FrontRightMotor.getPower();
        double rr = m_RearRightMotor.getPower();

        double delta = ((fl + rl) - (fr + rr));

        double a = fl - delta;
        double b = fr - delta;

        if (a == 0 && b == 0) {
            return Double.NaN;
        } else {
            return Math.atan2(b, a)-Math.PI/4;
        }
    }
}