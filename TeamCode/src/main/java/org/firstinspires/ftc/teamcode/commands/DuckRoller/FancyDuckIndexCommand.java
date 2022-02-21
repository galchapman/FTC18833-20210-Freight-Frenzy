package org.firstinspires.ftc.teamcode.commands.DuckRoller;

import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FancyDuckIndexCommand extends SequentialCommandGroup {
    public FancyDuckIndexCommand(DucksSubsystem ducksSubsystem, double rotations, double power0, double power1, double wait) {
        super(
                new IndexDuckCommand(ducksSubsystem, rotations, power0).withTimeout(wait),
                new InstantCommand(() -> ducksSubsystem.setPower(power1)),
                new WaitUntilCommand(() -> !ducksSubsystem.isBusy())
        );
    }
}
