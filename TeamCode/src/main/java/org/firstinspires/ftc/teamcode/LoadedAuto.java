package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.lib.auto.AutoLoader;
import org.firstinspires.ftc.teamcode.lib.auto.commands.EnumArgCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.SetCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.SingleArgCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class LoadedAuto extends BaseAuto {
    private AutoLoader autoLoader;
    private final String file;

    public LoadedAuto(String file) {
        this.file = file;
    }

    @Override
    public void initialize() {
        autoLoader = new AutoLoader();

        autoLoader.interpreter.registerCommand("door",
                new EnumArgCommand<>(IntakeSubsystem.DoorState.class,
                        (doorState) -> new InstantCommand(
                                () -> intakeSubsystem.setDoorState(doorState))));
        autoLoader.interpreter.registerCommand("wait", new SingleArgCommand(WaitCommand::new));
        autoLoader.interpreter.registerCommand("turn", new SingleArgCommand((angle) -> new TurnCommand(driveTrain, angle)));
        autoLoader.interpreter.registerCommand("set", new SetCommand());

        autoLoader.interpreter.env.addVariable("intake.height", (double value) -> armSubsystem.setVerticalPosition(value));

        telemetry.addData("external heading", () -> Math.toDegrees(driveTrain.getExternalHeading()));
        telemetry.addData("l pos", driveTrain::getFrontLeftPosition);
        telemetry.addData("r pos", driveTrain::getFrontRightPosition);
        telemetry.addData("h pos", driveTrain::getHorizontalPosition);
    }

    @Override
    public Command getAutonomousCommand() throws Exception {
        return autoLoader.load(file).andThen(() -> driveTrain.stop());
    }
}
