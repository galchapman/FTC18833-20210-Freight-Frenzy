package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StraitDriveCommand extends CommandBase {
    private final TankDrive m_drive;
    private final DoubleSupplier m_supplier;

    public StraitDriveCommand(TankDrive drive, DoubleSupplier supplier) {
        m_drive = drive;
        m_supplier = supplier;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double power = m_supplier.getAsDouble();
        m_drive.tankDrive(power, power);
    }
}
