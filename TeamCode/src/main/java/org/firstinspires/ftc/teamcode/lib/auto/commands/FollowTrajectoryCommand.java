package org.firstinspires.ftc.teamcode.lib.auto.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import java.util.List;
import java.util.Map;

import edu.megiddo.lions.Environment;
import edu.megiddo.lions.Tokenizer;
import edu.megiddo.lions.execption.FunctionException;
import edu.megiddo.lions.execption.LanguageException;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTrajectoryCommand implements edu.megiddo.lions.commands.Command<Command> {
    private final DriveTrainSubsystem m_driveTrain;
    private final Map<String, Trajectory> m_trajectories;

    public FollowTrajectoryCommand(DriveTrainSubsystem driveTrain, Map<String, Trajectory> trajectories) {
        m_driveTrain = driveTrain;
        m_trajectories = trajectories;
    }

    @Override
    public Command run(Environment env, Tokenizer.Token function, List<Tokenizer.Token> args) throws LanguageException {
        if (args.size() != 1) {
            throw new FunctionException("Invalid arg amount", function);
        }

        return new org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand(
                m_driveTrain, m_trajectories.get(args.get(0).value));
    }
}
