package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.IntakeMineralCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.drive.RoadRunnerThread;
import org.firstinspires.ftc.teamcode.commands.drive.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.lib.DashboardUtil;
import org.firstinspires.ftc.teamcode.lib.StartingPosition;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class BaseAuto extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected LiftSubsystem liftSubsystem;
    protected DucksSubsystem ducksSubsystem;
    protected VisionSubsystem vision;
    protected StartingPosition startingPosition;

    protected RoadRunnerThread thread;

    protected BaseAuto(StartingPosition startingPosition) {
        this.startingPosition = startingPosition;
    }
    private double lastTime = 0;

    @Override
    public void plan() {
        RobotUniversal.opModeType = RobotUniversal.OpModeType.Autonomous;
        RobotUniversal.startingPosition = startingPosition;
        driveTrain = new DriveTrainSubsystem();
        armSubsystem = new ArmSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        liftSubsystem = new LiftSubsystem();
        vision = new VisionSubsystem();
        ducksSubsystem = new DucksSubsystem();

        thread = new RoadRunnerThread(driveTrain);

        driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);
        armSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSubsystem.setPower(1);

        liftSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSubsystem.setPower(1);

        initialize();

        TelemetryPacket temp_packet = new TelemetryPacket();
        DashboardUtil.drawRobot(temp_packet.fieldOverlay(), driveTrain.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(temp_packet);

        RobotUniversal.telemetryPacketUpdater = (packet) -> {
            packet.put("FrontLeftPower", driveTrain.getFrontLeftPower());
            packet.put("RearLeftPower", driveTrain.getRearLeftPower());
            packet.put("FrontRightPower", driveTrain.getFrontRightPower());
            packet.put("RearRightPower", driveTrain.getRearRightPower());
            packet.put("isBusy", driveTrain.isBusy());
            double time = getRuntime();
            packet.put("dt", time - lastTime);
            lastTime = time;

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        };
    }

    @Override
    public void onStart() {
        thread.schedule();
        vision.stop();
        armSubsystem.setVerticalPosition(1);
    }

    @Override
    public void onEnd() {
        RobotUniversal.endPosition = driveTrain.getPoseEstimate();
    }

    abstract public void initialize();

    protected Command follow(Trajectory trajectory) {
        return new FollowTrajectoryCommand(driveTrain, trajectory);
    }

    protected Command follow(TrajectorySequence trajectory) {
        return new FollowTrajectorySequenceCommand(driveTrain, trajectory);
    }

    protected Command forward(double distance) {
        return new DriveForwardCommand(driveTrain, driveTrain::getLeftPosition, driveTrain::getRightPosition, distance);
    }

    protected Command turn(double angle) {
        return new TurnCommand(driveTrain, angle);
    }

    protected Command strafe(double distance) {
        return new StrafeCommand(driveTrain, distance);
    }

    protected Command setDoor(IntakeSubsystem.DoorState state) {
        return new InstantCommand(() -> intakeSubsystem.setDoorState(state));
    }

    protected Command intake(double distance) {
        return new IntakeMineralCommand(driveTrain, armSubsystem, intakeSubsystem, distance);
    }
}
