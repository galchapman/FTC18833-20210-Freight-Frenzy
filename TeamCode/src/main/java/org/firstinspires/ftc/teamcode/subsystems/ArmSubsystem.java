package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class ArmSubsystem extends SubsystemBase {
	private final DcMotor m_motor;
	private final Servo m_leftServo;
	private final Servo m_rightServo;

	public ArmSubsystem() {
		m_motor = hardwareMap.dcMotor.get("ArmMotor");
		m_leftServo = hardwareMap.servo.get("LeftIntakeArmServo");
		m_rightServo = hardwareMap.servo.get("RightIntakeArmServo");
		m_motor.setDirection(DcMotorSimple.Direction.REVERSE);
		m_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		m_motor.setPower(0);
		m_motor.setTargetPosition(0);
//		m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		m_leftServo.setDirection(Servo.Direction.REVERSE);
	}

	public void setRunMode(DcMotor.RunMode mode) {
		m_motor.setMode(mode);
	}

	public void setPower(double power) {
		m_motor.setPower(power);
	}

	public double getPower() {
		return m_motor.getPower();
	}

	public void stop() {
		m_motor.setPower(0);
	}

	public void setTargetPosition(int position) {
		m_motor.setTargetPosition(position);
	}

	public double getAngle() {
		return (double)m_motor.getCurrentPosition() / ArmConstants.motorGear / ArmConstants.gear * 360;
	}

	public double getTargetAngle() {
		return (double)m_motor.getTargetPosition() / ArmConstants.motorGear / ArmConstants.gear * 360;
	}

	public void setAngle(double angle) {
		m_motor.setTargetPosition((int)(angle * ArmConstants.motorGear * ArmConstants.gear / 360));
	}

	public void setVerticalPosition(double position) {
		m_leftServo.setPosition(position);
		m_rightServo.setPosition(position);
	}

	public double getVerticalPosition() {
		return m_leftServo.getPosition();
	}

	public double SyncError() {
		return Math.abs(m_leftServo.getPosition() - m_rightServo.getPosition());
	}

	public boolean isBusy() {
		return m_motor.isBusy();
	}

	public double AngleError() {
		return getAngle() - getTargetAngle();
	}

	public int getTargetPosition() {
		return m_motor.getTargetPosition();
	}

	public int getCurrentPosition() {
		return m_motor.getCurrentPosition();
	}
}
