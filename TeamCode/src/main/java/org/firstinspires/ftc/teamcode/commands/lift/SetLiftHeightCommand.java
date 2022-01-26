package org.firstinspires.ftc.teamcode.commands.lift;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLiftHeightCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem;
    private final double m_height;
    private final double m_power;

    public  SetLiftHeightCommand(LiftSubsystem liftSubsystem, double height, double power){
        m_liftSubsystem = liftSubsystem;
        m_height = height;
        m_power = power;

        addRequirements(m_liftSubsystem);
    }

    @Override
    public void initialize() {
        m_liftSubsystem.setLiftHeight(m_height);
        m_liftSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_liftSubsystem.setPower(m_power);
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.stop();
        m_liftSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean isFinished() {return !m_liftSubsystem.isBusy();}
}
