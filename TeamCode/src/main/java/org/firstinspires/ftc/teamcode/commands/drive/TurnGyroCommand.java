package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnGyroCommand extends CommandBase {
    private final TankDrive m_drive;
    private final DoubleSupplier m_angleSupplier;
    private final double m_target;
    private final double m_power;
    private double lastAngle;
    private double lastRawAngle;
    private int spinCount;
    private double error;
    private double target;

    public TurnGyroCommand(TankDrive drive, DoubleSupplier angleSupplier, double target, double power) {
        m_drive = drive;
        m_angleSupplier = angleSupplier;
        m_target = target;
        m_power = power;

        addRequirements(drive);
    }

    public double getError() {
        return error;
    }

    public int getSpinCount() {
        return spinCount;
    }

    public double lastAngle() {
        return lastAngle;
    }

    private void spin(double power) {
        m_drive.tankDrive(-power, power);
    }

    @Override
    public void initialize() {
        lastRawAngle = m_angleSupplier.getAsDouble();
        spinCount = 0;
        target = lastRawAngle + m_target;
    }

    public double delta = 0;

    @Override
    public void execute() {
        double angle = m_angleSupplier.getAsDouble();
        delta = angle - lastRawAngle;
        lastRawAngle = angle;

        if (delta > Math.PI / 2) {
            spinCount--;
        } else if (delta < -Math.PI / 2) {
            spinCount++;
        }
        angle += spinCount * 2 * Math.PI;

        error = m_target - angle;

        spin(Math.abs(error) > 0.05 ? m_power * Util.clamp(-1, 1, error * 2) : 0.4);

        lastAngle = angle;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < 0.03 && Math.abs(delta) < 0.025;
    }
}
