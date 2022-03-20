package org.firstinspires.ftc.teamcode.commands.drive;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Just like DriveRight, but this allows diagonal movement.
 */
public class GeneralDriveRightCommand extends CommandBase {
    private final DriveTrainSubsystem m_driveTrain;
    private final DoubleSupplier m_verticalSupplier;

    public GeneralDriveRightCommand(DriveTrainSubsystem driveTrain, DoubleSupplier verticalSupplier) {
        this.m_driveTrain = driveTrain;
        this.m_verticalSupplier = verticalSupplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double power = m_verticalSupplier.getAsDouble();
        if (Math.abs(power) > 0.5) {
            if (power > 0) {
                m_driveTrain.setPowers(1, 0, 0, 1);
            } else {
                m_driveTrain.setPowers(0, -1, -1, 0);
            }
        } else {
            m_driveTrain.driveRight(1);
        }
    }
}
