package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final DoubleSupplier m_supplier;
    private final double m_power;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier supplier) {
        m_intake = intake;
        m_supplier = supplier;
        m_power = 0;

        addRequirements(m_intake);
    }

    public IntakeCommand(IntakeSubsystem intake, double power) {
        m_intake = intake;
        m_supplier = null;
        m_power = power;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        if (m_supplier != null)
            m_intake.intake(m_supplier.getAsDouble());
        else
            m_intake.intake(m_power);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
