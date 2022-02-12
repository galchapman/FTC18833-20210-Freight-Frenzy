package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.auto.AutoLoader;
import org.firstinspires.ftc.teamcode.lib.auto.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.lib.auto.commands.EnumArgCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.SetCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.SingleArgCommand;
import org.firstinspires.ftc.teamcode.lib.auto.commands.StaticSetCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class LoadedAuto extends BaseAuto {
    private AutoLoader autoLoader;
    private final String file;
    private final String trajectoriesPath;
    private final Pose2d start;
    TrajectoryLoader trajectoryLoader;
    Map<String, Trajectory> trajectories;
    boolean A = false, B = false;

    public LoadedAuto(String file, String trajectoriesPath, Pose2d start) {
        this.file = file;
        this.trajectoriesPath = trajectoriesPath;
        this.start = start;
    }

    @Override
    public void initialize() {
        autoLoader = new AutoLoader();
        trajectoryLoader = new TrajectoryLoader(trajectoriesPath);
        driveTrain.setPoseEstimate(start);

        trajectories = trajectoryLoader.getTrajectories(driveTrain::trajectoryBuilder);

        autoLoader.interpreter.registerCommand("door",
                new EnumArgCommand<>(IntakeSubsystem.DoorState.class,
                        (doorState) -> new InstantCommand(
                                () -> intakeSubsystem.setDoorState(doorState))));
        autoLoader.interpreter.registerCommand("wait", new SingleArgCommand(WaitCommand::new));
        autoLoader.interpreter.registerCommand("turn", new SingleArgCommand(this::turn));
        autoLoader.interpreter.registerCommand("forward", new SingleArgCommand(this::forward));
        autoLoader.interpreter.registerCommand("set", new SetCommand());
        autoLoader.interpreter.registerCommand("set.static", new StaticSetCommand<>());
        autoLoader.interpreter.registerCommand("follow", new FollowTrajectoryCommand(driveTrain, trajectories));
        autoLoader.interpreter.registerCommand("arm.mode", new EnumArgCommand<>(DcMotor.RunMode.class,
                (mode) -> new InstantCommand(() -> armSubsystem.setRunMode(mode))));

        autoLoader.interpreter.registerCommand("door", new EnumArgCommand<>(IntakeSubsystem.DoorState.class, this::setDoor));

        autoLoader.interpreter.env.addVariable("intake.height", armSubsystem::setVerticalPosition);
        autoLoader.interpreter.env.addVariable("lift.height", liftSubsystem::setLiftHeight);

        autoLoader.interpreter.env.addVariable("A", () -> A);
        autoLoader.interpreter.env.addVariable("A", (Boolean value) -> A = value);
        autoLoader.interpreter.env.addVariable("B", () -> B);
        autoLoader.interpreter.env.addVariable("B", (Boolean value) -> B = value);

        telemetry.addData("external heading", () -> Math.toDegrees(driveTrain.getExternalHeading()));
    }

    @Override
    public Command getAutonomousCommand() throws Exception {
        return autoLoader.load(file).andThen(() -> driveTrain.stop());
    }
}
