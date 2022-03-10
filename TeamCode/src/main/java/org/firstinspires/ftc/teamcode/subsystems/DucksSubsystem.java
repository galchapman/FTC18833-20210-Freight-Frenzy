package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class DucksSubsystem extends SubsystemBase {
    private final DcMotor m_motor;

    public DucksSubsystem() {
        m_motor = hardwareMap.dcMotor.get("DucksMotor");
        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_motor.setPower(0);
        m_motor.setTargetPosition(0);
        m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        m_motor.setPower(power);
    }

    public void setRunMode(DcMotor.RunMode mode){m_motor.setMode(mode);}

    public void setDirection(DcMotorSimple.Direction direction){m_motor.setDirection(direction);}

    public boolean isBusy() {
        return m_motor.isBusy();
    }

    public double getPower() {
        return m_motor.getPower();
    }

    public int getCurrentPosition() {
        return m_motor.getCurrentPosition();
    }

    public void spin(double rotations) {
        m_motor.setTargetPosition(m_motor.getCurrentPosition() + (int)(rotations * Constants.DucksConstants.ticks_per_rotation));
    }

    public void stop() {
        m_motor.setTargetPosition(m_motor.getCurrentPosition());
        m_motor.setPower(0);
    }

    public int getError() {
        return m_motor.getTargetPosition() - m_motor.getCurrentPosition();
    }
}
