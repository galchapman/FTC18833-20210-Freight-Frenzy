package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnGyroCommand extends CommandBase {
    private final TankDrive m_drive;
    private final DoubleSupplier m_angleSupplier;
    private final double m_target;
    private final double m_power;
    private double lastAngle;
    private int spinCount;
    private double error;

    public TurnGyroCommand(TankDrive drive, DoubleSupplier angleSupplier, double target, double power) {
        m_drive = drive;
        m_angleSupplier = angleSupplier;
        m_target = target;
        m_power = power;

        addRequirements(drive);
    }

    private void spin(double power) {
        m_drive.tankDrive(power, -power);
    }

    @Override
    public void initialize() {
        lastAngle = m_angleSupplier.getAsDouble();
        spinCount = 0;
    }

    @Override
    public void execute() {
        double angle = m_angleSupplier.getAsDouble();
        if (Math.abs(angle - lastAngle) > Math.PI ) {
            spinCount++;
        }
        angle += spinCount * 2 * Math.PI;

        error = m_target - angle;

        spin(error);

        lastAngle = angle;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < 0.05;
    }
}
