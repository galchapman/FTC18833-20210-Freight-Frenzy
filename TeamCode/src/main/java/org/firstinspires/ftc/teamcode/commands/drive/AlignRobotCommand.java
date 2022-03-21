package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignRobotCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final DoubleSupplier m_forwardSupplier;
    private final double m_left;
    private final double m_target;

    public AlignRobotCommand(DriveTrainSubsystem driveTrain, DoubleSupplier forwardSupplier, double left, double target) {
        m_driveTrain = driveTrain;
        m_forwardSupplier = forwardSupplier;
        m_target = target;
        m_left = left;

        addRequirements(m_driveTrain);
    }

    @Override
    public void execute() {
        double headingError = m_driveTrain.getHeading() - m_target;
        m_driveTrain.arcadeDrive(m_left, m_forwardSupplier.getAsDouble(), Math.abs(headingError) < Math.toRadians(5) ? 0
                : headingError * 0.3);
    }
}
