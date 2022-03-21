package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDriveCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_spinSupplier;
    private final DoubleSupplier m_speedSupplier;

    public ArcadeDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier spinSupplier, double power) {
        this(driveTrain, xSupplier, ySupplier, spinSupplier, () -> power);
    }

    public ArcadeDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier spinSupplier, DoubleSupplier speedSupplier) {
        m_driveTrain = driveTrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_spinSupplier = spinSupplier;
        m_speedSupplier = speedSupplier;

        addRequirements(m_driveTrain);
    }

    public ArcadeDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier spinSupplier) {
        this(driveTrain, xSupplier, ySupplier, spinSupplier, 1);
    }

    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble(), m_spinSupplier.getAsDouble(), m_speedSupplier.getAsDouble());
    }
}
