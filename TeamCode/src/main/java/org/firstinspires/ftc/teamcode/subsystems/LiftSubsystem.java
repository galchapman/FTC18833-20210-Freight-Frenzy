package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.LiftConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotor m_liftMotor;
    private final DistanceSensor m_heightSensor;
    private final int m_encoderOffset;

    public LiftSubsystem() {
        m_liftMotor = hardwareMap.dcMotor.get("LiftMotor");
        m_heightSensor = hardwareMap.get(DistanceSensor.class, "LiftHeightSensor");
        m_liftMotor.setPower(0);
        m_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_liftMotor.setTargetPosition(0);
        m_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_encoderOffset = meters2ticks(getSensorHeight() - 0.080);

        System.out.println("Some random print Height: " +  m_encoderOffset);
        System.out.println("Some random print Height: " +  ticks2meters(m_encoderOffset));
    }

    public double getSensorHeight() {
        return m_heightSensor.getDistance(DistanceUnit.METER);
    }

    public int getCurrentPosition() {
        return m_liftMotor.getCurrentPosition();
    }
    public int getTargetPosition() {
        return m_liftMotor.getTargetPosition();
    }

    public double getHeight() {
        return ticks2meters(m_liftMotor.getCurrentPosition() + m_encoderOffset);
    }

    public void setLiftHeight(double height) {
            m_liftMotor.setTargetPosition(meters2ticks(height) - m_encoderOffset);
    }

    static public int meters2ticks(double height) {
        return (int)(height/ LiftConstants.distance_per_tick);
    }

    static public double ticks2meters(int ticks) {
        return ticks * LiftConstants.distance_per_tick;
    }


    public void setRunMode(DcMotor.RunMode mode) {m_liftMotor.setMode(mode);}

    public void setPower(double power) {
        m_liftMotor.setPower(power);
    }

    public double getPower() {
        return m_liftMotor.getPower();
    }

    public void stop() {
        m_liftMotor.setPower(0);
    }

    public boolean isBusy() {
        return m_liftMotor.isBusy();
    }

    public int getEncoderOffset() {
        return m_encoderOffset;
    }
}
