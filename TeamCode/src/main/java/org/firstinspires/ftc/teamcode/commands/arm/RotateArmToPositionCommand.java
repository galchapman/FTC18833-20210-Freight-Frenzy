package org.firstinspires.ftc.teamcode.commands.arm;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateArmToPositionCommand extends CommandBase {
    public final ArmSubsystem m_ArmSubsystem;
    private final int m_angle;
    private final double m_power;

    public RotateArmToPositionCommand(ArmSubsystem armSubsystem, int angle ,double power) {
        m_ArmSubsystem = armSubsystem;
        m_angle = angle;
        m_power = power;
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        //m_ArmSubsystem.setTargetPosition(m_ArmSubsystem.getCurrentPosition() + m_rotation);
        m_ArmSubsystem.setAngle(m_angle);
        m_ArmSubsystem.setPower(m_power);
    }

    @Override
    public boolean isFinished() {
        return m_ArmSubsystem.isBusy();
    }

}
