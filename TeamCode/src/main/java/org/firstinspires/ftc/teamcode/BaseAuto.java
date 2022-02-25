package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.drive.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnGyroCommand;
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

    protected BaseAuto(StartingPosition startingPosition) {
        this.startingPosition = startingPosition;
    }

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

        driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);
        armSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSubsystem.setPower(1);

        liftSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSubsystem.setPower(1);

        initialize();

        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawRobot(packet.fieldOverlay(), driveTrain.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("height", liftSubsystem::getHeight);
    }

    @Override
    public void onStart() {
        vision.stop();

        armSubsystem.setVerticalPosition(1);
    }

    abstract public void initialize();

    protected Command follow(Trajectory trajectory) {
        return new FollowTrajectoryCommand(driveTrain, trajectory);
    }

    protected Command follow(TrajectorySequence trajectory) {
        return new FollowTrajectorySequenceCommand(driveTrain, trajectory);
    }

    protected Command forward(double distance) {
        return new DriveForwardCommand(driveTrain, driveTrain::getFrontLeftPosition, driveTrain::getFrontRightPosition, distance);
    }

    protected Command turn(double angle) {
        return new TurnGyroCommand(driveTrain, driveTrain::getHeading, angle, 1).andThen(new WaitCommand(1));
    }

    protected Command strafe(double distance) {
        return new StrafeCommand(driveTrain, distance);
    }

    protected Command setDoor(IntakeSubsystem.DoorState state) {
        return new InstantCommand(() -> intakeSubsystem.setDoorState(state));
    }
}
