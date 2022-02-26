package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.drive.GeneralDriveLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.GeneralDriveRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.StraitDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnGyroCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@TeleOp
public class OdometryTest extends CommandBasedTeleOp {
    DriveTrainSubsystem driveTrain;
    ArmSubsystem armSubsystem;

    SequentialCommandGroup fastTest;
    SequentialCommandGroup slowTest;

    @Override
    public void assign() {
        RobotUniversal.setOpMode(this);
        RobotUniversal.opModeType = RobotUniversal.OpModeType.TeleOp;

        driveTrain = new DriveTrainSubsystem();
        armSubsystem = new ArmSubsystem();

        armSubsystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSubsystem.setPower(1);
        armSubsystem.setVerticalPosition(1);

        driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y));

        driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrain.setOdometryPosition(DriveTrainSubsystem.OdometryPosition.Down);

        fastTest = new TurnGyroCommand(driveTrain, driveTrain::getHeading, Math.toRadians(360) * 5, 1).andThen(
                new WaitCommand(2),
                new TurnGyroCommand(driveTrain, driveTrain::getHeading, -Math.toRadians(360) * 5, 1)
        );
        slowTest = new TurnGyroCommand(driveTrain, driveTrain::getHeading, Math.toRadians(360) * 5, 0.7).andThen(
                new WaitCommand(2),
                new TurnGyroCommand(driveTrain, driveTrain::getHeading, -Math.toRadians(360) * 5, 0.7)
        );

        gp1.y().whenPressed(fastTest);
        gp1.x().whenPressed(slowTest);

        gp1.left_stick_button().whileHeld(new StraitDriveCommand(driveTrain, () -> -gamepad1.left_stick_y));
        gp1.right_stick_button().whileHeld(new StraitDriveCommand(driveTrain, () -> -gamepad1.right_stick_y));

        gp1.left_bumper().whileHeld(new GeneralDriveLeftCommand(driveTrain, () -> -gamepad1.left_stick_y));
        gp1.right_bumper().whileHeld(new GeneralDriveRightCommand(driveTrain, () -> -gamepad1.right_stick_y));

        RobotUniversal.telemetryPacketUpdater = (packet) -> {
            packet.put("left", driveTrain.getFrontLeftPosition());
            packet.put("right", driveTrain.getFrontRightPosition());
            packet.put("left distance", driveTrain.getLeftDistance());
            packet.put("right distance", driveTrain.getRightDistance());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        };
    }
}
