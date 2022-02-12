package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.drive.TurnGyroCommand;
import org.firstinspires.ftc.teamcode.lib.auto.AutoLoader;
import org.firstinspires.ftc.teamcode.lib.auto.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.lib.auto.commands.EnumArgCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.SetCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.SingleArgCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class LoadedAuto extends BaseAuto {
    private AutoLoader autoLoader;
    private final String file;
    private final String trajectoriesPath;
    TrajectoryLoader trajectoryLoader;
    Map<String, Trajectory> trajectories;

    public LoadedAuto(String file, String trajectoriesPath) {
        this.file = file;
        this.trajectoriesPath = trajectoriesPath;
    }

    @Override
    public void initialize() {
        autoLoader = new AutoLoader();
        trajectoryLoader = new TrajectoryLoader(trajectoriesPath);

        trajectories = trajectoryLoader.getTrajectories(driveTrain::trajectoryBuilder);

        autoLoader.interpreter.registerCommand("door",
                new EnumArgCommand<>(IntakeSubsystem.DoorState.class,
                        (doorState) -> new InstantCommand(
                                () -> intakeSubsystem.setDoorState(doorState))));
        autoLoader.interpreter.registerCommand("wait", new SingleArgCommand(WaitCommand::new));
        autoLoader.interpreter.registerCommand("turn", new SingleArgCommand((angle) -> new TurnGyroCommand(driveTrain, driveTrain::getHeading, angle, 1)));
        autoLoader.interpreter.registerCommand("forward", new SingleArgCommand(this::forward));
        autoLoader.interpreter.registerCommand("set", new SetCommand());
        autoLoader.interpreter.registerCommand("follow", new FollowTrajectoryCommand(driveTrain, trajectories));
        autoLoader.interpreter.registerCommand("arm.mode", new EnumArgCommand<>(DcMotor.RunMode.class,
                (mode) -> new InstantCommand(() -> armSubsystem.setRunMode(mode))));

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
