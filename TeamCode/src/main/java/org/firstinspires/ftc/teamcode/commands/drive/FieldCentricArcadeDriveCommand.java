package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FieldCentricArcadeDriveCommand extends CommandBase {
    private final DriveTrainSubsystem m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_spinSupplier;
    private final double m_angleOffset;

    public FieldCentricArcadeDriveCommand(DriveTrainSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier spinSupplier, double angleOffset) {
        m_drive = drive;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_spinSupplier = spinSupplier;
        m_angleOffset = angleOffset;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Vector2d input = new Vector2d(
                -m_xSupplier.getAsDouble(),
                -m_ySupplier.getAsDouble()
        ).rotated(-m_drive.getHeading()+m_angleOffset);

        m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        m_spinSupplier.getAsDouble()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
