package org.firstinspires.ftc.teamcode.commands.arm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateArmToPositionCommand extends CommandBase {
    public final ArmSubsystem m_ArmSubsystem;
    private final double m_angle;
    private final double m_power;

    public RotateArmToPositionCommand(ArmSubsystem armSubsystem, double angle ,double power) {
        m_ArmSubsystem = armSubsystem;
        m_angle = angle;
        m_power = power;
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.setAngle(m_angle);
        m_ArmSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_ArmSubsystem.setPower(m_power);
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return m_ArmSubsystem.isBusy() && Math.abs(m_ArmSubsystem.AngleError()) < 3;
    }

}
