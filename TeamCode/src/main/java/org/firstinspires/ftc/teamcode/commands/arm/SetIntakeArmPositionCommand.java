package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeArmPositionCommand extends CommandBase {
    public final ArmSubsystem m_armSubsystem;
    private final double m_rotation;

    public SetIntakeArmPositionCommand(ArmSubsystem armSubsystem, double rotation) {
        m_armSubsystem = armSubsystem;
        m_rotation = rotation;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.setVerticalPosition(m_rotation);
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.getVerticalPosition() == m_rotation;
    }

}
