package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.arm.StopArmCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetRobotArmsPosition extends SequentialCommandGroup {

    private static class RotateArmCommand extends CommandBase {
        private final ArmSubsystem m_ArmSubsystem;
        private double m_angle;
        private final double m_power;

        public RotateArmCommand(ArmSubsystem armSubsystem, double angle ,double power) {
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
            return !m_ArmSubsystem.isBusy();
        }
    }

    private static class SetLiftHeightCommand extends CommandBase {
        private final LiftSubsystem m_liftSubsystem;
        private double m_height;
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
        public boolean isFinished() {return !m_liftSubsystem.isBusy();}
    }

    private final SetLiftHeightCommand m_setLiftHeightCommand;
    private final RotateArmCommand m_rotateArmCommand;
    private double m_targetIntakePosition;

    public SetRobotArmsPosition(ArmSubsystem armSubsystem, LiftSubsystem liftSubsystem,
                                double liftHeight, double liftPower,
                                double armAngle, double armPower,
                                double intakePosition) {
        m_rotateArmCommand = new RotateArmCommand(armSubsystem, armAngle, armPower);
        m_setLiftHeightCommand = new SetLiftHeightCommand(liftSubsystem, liftHeight, liftPower);

        m_targetIntakePosition = intakePosition;

        ParallelCommandGroup command = new ParallelCommandGroup(
                        m_setLiftHeightCommand,
                        m_rotateArmCommand.withTimeout(0.5).withInterrupt(() -> Math.abs(armSubsystem.AngleError()) < 10 && Math.abs(armSubsystem.getAngleVelocity()) < 50)
                                .andThen(new StopArmCommand(armSubsystem).withTimeout(0.7))
                );

        addCommands(
                new InstantCommand(() -> armSubsystem.setVerticalPosition(1)),
                command.withInterrupt(() -> Math.abs(liftSubsystem.getHeight() - m_setLiftHeightCommand.m_height) < 0.1)
        );

        if (!Double.isNaN(m_targetIntakePosition)) {
            addCommands(
                    new InstantCommand(() -> armSubsystem.setVerticalPosition(m_targetIntakePosition)));
        }

        addCommands(new WaitUntilCommand(() -> !liftSubsystem.isBusy()));
    }

    public void setTarget(double liftHeight, double armAngle, double intakePosition) {
        m_setLiftHeightCommand.m_height = liftHeight;
        m_rotateArmCommand.m_angle = armAngle;
        m_targetIntakePosition = intakePosition;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_setLiftHeightCommand.m_liftSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_setLiftHeightCommand.m_liftSubsystem.stop();
    }
}
