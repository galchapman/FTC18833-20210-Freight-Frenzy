package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor m_intakeMotor;
    private final Servo m_doorServo;
    private final DistanceSensor m_distanceSensor;

    public enum DoorState {
        Open(0.2),
        Close(1),
        LowerPlacement(0.2);

        public double servoPosition;
        DoorState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public IntakeSubsystem() {
        m_intakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        m_doorServo = hardwareMap.servo.get("IntakeDoorServo");
        m_distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "IntakeSensor");

        m_intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDoorState(DoorState.Close);
    }

    public void intake(double power) {
        m_intakeMotor.setPower(power);
    }

    public double getIntakePower() {
        return m_intakeMotor.getPower();
    }

    public void stop() {
        m_intakeMotor.setPower(0);
    }

    public void setDoorState(DoorState state) {
        m_doorServo.setPosition(state.servoPosition);
    }

    public void toggleDoor() {
        m_doorServo.setPosition((DoorState.Close.servoPosition + DoorState.Open.servoPosition) - m_doorServo.getPosition());
    }

    public double getDistance() {
        return m_distanceSensor.getDistance(DistanceUnit.METER);
    }

    public boolean hasFreight() {
        return getDistance() < IntakeConstants.intakeThreshold;
    }
}
