package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDriveCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_spinSupplier;

    public ArcadeDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier spinSupplier) {
        m_driveTrain = driveTrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_spinSupplier = spinSupplier;

        addRequirements(m_driveTrain);
    }

    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble(), m_spinSupplier.getAsDouble());
    }
}
