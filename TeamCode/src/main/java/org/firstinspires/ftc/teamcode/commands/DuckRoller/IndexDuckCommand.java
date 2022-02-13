package org.firstinspires.ftc.teamcode.commands.DuckRoller;

import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexDuckCommand extends CommandBase {
    private final DucksSubsystem m_duckSubsystem;
    private final double m_rotations;
    private final double m_power;

    public IndexDuckCommand(DucksSubsystem duckSubsystem, double rotations, double power) {
        m_duckSubsystem = duckSubsystem;
        m_rotations = rotations;
        m_power = power;

        addRequirements(duckSubsystem);
    }

    @Override
    public void initialize() {
        m_duckSubsystem.spin(m_rotations);
        m_duckSubsystem.setPower(m_power);
    }

    @Override
    public boolean isFinished() {
        return m_duckSubsystem.isBusy();
    }
}
