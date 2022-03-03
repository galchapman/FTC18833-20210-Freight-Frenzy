package org.firstinspires.ftc.teamcode.commands.DuckRoller;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FancyDuckIndexCommand extends CommandBase {

    private final DucksSubsystem m_ducksSubsystem;
    private final double MaxSpeed;
    private final double MinSpeed;
    private double start_time;
    private final double accelerationSpeed;

    public FancyDuckIndexCommand(DucksSubsystem ducksSubsystem, double MinSpeed, double MaxSpeed, double accelerationSpeed) {
        m_ducksSubsystem = ducksSubsystem;
        this.accelerationSpeed = accelerationSpeed;
        this.MaxSpeed = MaxSpeed;
        this.MinSpeed = MinSpeed;

        addRequirements(m_ducksSubsystem);
    }

    @Override
    public void initialize() {
        m_ducksSubsystem.spin(-1.3);
        m_ducksSubsystem.setPower(MinSpeed);
        start_time = RobotUniversal.opMode.getRuntime();
    }

    @Override
    public void execute() {
        m_ducksSubsystem.setPower(Math.min(MaxSpeed,
                MinSpeed + (RobotUniversal.opMode.getRuntime() - start_time) * accelerationSpeed));
    }

    @Override
    public boolean isFinished() {
        return !m_ducksSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        m_ducksSubsystem.setPower(0);
    }
}
