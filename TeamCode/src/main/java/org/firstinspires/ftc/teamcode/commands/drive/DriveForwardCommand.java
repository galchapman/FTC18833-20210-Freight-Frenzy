package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardCommand extends CommandBase {
    private final TankDrive m_drive;
    private final DoubleSupplier m_leftDistance;
    private final DoubleSupplier m_rightDistance;
    private final double m_distance;
    private double targetLeft;
    private double targetRight;
    private double lastLeft;
    private double lastRight;
    private double deltaLeft;
    private double deltaRight;
    private double leftError;
    private double rightError;

    public DriveForwardCommand(TankDrive drive, DoubleSupplier leftDistance, DoubleSupplier rightDistance, double distance) {
        m_drive = drive;
        m_leftDistance = leftDistance;
        m_rightDistance = rightDistance;
        m_distance = distance;
    }

    @Override
    public void initialize() {
        targetLeft = m_leftDistance.getAsDouble() + m_distance;
        targetRight = m_rightDistance.getAsDouble() + m_distance;
    }

    @Override
    public void execute() {
        double left = m_leftDistance.getAsDouble(),
                right = m_rightDistance.getAsDouble();

        deltaLeft = left - lastLeft;
        deltaRight = right - lastRight;

        leftError = targetLeft - left;
        rightError = targetRight - right;

        double power = Util.maxAbs(leftError, rightError) * 2;
        double correction = (leftError - rightError) / 3;
        m_drive.tankDrive(power + correction, power - correction);

        lastLeft = left;
        lastRight = right;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.max(Math.abs(leftError), Math.abs(rightError)) < 0.02
                && Math.max(Math.abs(deltaLeft), Math.abs(deltaRight)) < 0.01;
    }
}
