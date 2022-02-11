package org.firstinspires.ftc.teamcode.commands.arm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopArmCommand extends CommandBase {
    private final ArmSubsystem m_armSubsystem;

    public StopArmCommand(ArmSubsystem armSubsystem) {
        this.m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.setTargetPosition(m_armSubsystem.getCurrentPosition());
        m_armSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_armSubsystem.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.setPower(0);
        m_armSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
