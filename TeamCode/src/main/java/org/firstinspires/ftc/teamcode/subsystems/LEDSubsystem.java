package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class LEDSubsystem extends SubsystemBase {
    private final RevBlinkinLedDriver m_driver;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    public LEDSubsystem() {
        m_driver = hardwareMap.get(RevBlinkinLedDriver.class, "LEDDriver");
        setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        m_driver.setPattern(currentPattern = pattern);
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return currentPattern;
    }

    public void next() {
        m_driver.setPattern(currentPattern = currentPattern.next());
    }


    public void previous() {
        m_driver.setPattern(currentPattern = currentPattern.previous());
    }
}
