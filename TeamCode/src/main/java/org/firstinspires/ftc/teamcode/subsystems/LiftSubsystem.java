package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.lib.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.LiftConstants.lower_plate_height;
import static org.firstinspires.ftc.teamcode.Constants.LiftConstants.top_height;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotor m_liftMotor;
    private final TouchSensor m_bottomLimitSwitch;
    private int m_encoderOffset;

    public LiftSubsystem() {
        m_liftMotor = hardwareMap.dcMotor.get("LiftMotor");
        m_bottomLimitSwitch = hardwareMap.touchSensor.get("LiftBottomLimitSwitch");
        m_liftMotor.setPower(0);
        m_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_liftMotor.setTargetPosition(0);
        m_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_encoderOffset = meters2ticks(lower_plate_height);
    }

    @Override
    public void periodic() {
        if (m_bottomLimitSwitch.isPressed()) {
            m_encoderOffset = meters2ticks(lower_plate_height) - m_liftMotor.getCurrentPosition();
            if (m_liftMotor.getTargetPosition() < m_liftMotor.getCurrentPosition())
                setLiftHeight(lower_plate_height);
        }
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

    public boolean isDown() {
        return m_bottomLimitSwitch.isPressed();
    }

    public boolean isUp() {
        return getHeight() > top_height;
    }

    public void setLiftHeight(double height) {
            m_liftMotor.setTargetPosition(meters2ticks(Util.clamp(lower_plate_height, top_height, height)) - m_encoderOffset);
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
        m_liftMotor.setTargetPosition(m_liftMotor.getCurrentPosition());
    }

    public boolean isBusy() {
        return m_liftMotor.isBusy();
    }

    public int getEncoderOffset() {
        return m_encoderOffset;
    }
}
