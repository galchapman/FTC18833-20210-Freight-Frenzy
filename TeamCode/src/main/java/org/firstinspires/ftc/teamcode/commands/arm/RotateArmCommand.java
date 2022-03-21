package org.firstinspires.ftc.teamcode.commands.arm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateArmCommand extends CommandBase {
    private final ArmSubsystem m_armSubsystem;
    private final DoubleSupplier m_supplier;

    public RotateArmCommand(ArmSubsystem armSubsystem, DoubleSupplier supplier) {
        m_armSubsystem = armSubsystem;
        m_supplier = supplier;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void execute() {
        m_armSubsystem.setPower(m_supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stop();
    }
}
