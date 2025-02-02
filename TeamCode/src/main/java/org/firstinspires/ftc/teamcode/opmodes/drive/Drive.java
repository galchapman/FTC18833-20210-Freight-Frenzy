package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.commandftc.RobotUniversal;
import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DuckRoller.FancyDuckIndexCommand;
import org.firstinspires.ftc.teamcode.commands.SetRobotArmsPosition;
import org.firstinspires.ftc.teamcode.commands.arm.RotateArmCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.arm.SetIntakeArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FieldCentricArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.GeneralDriveLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.GeneralDriveRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.lift.RaiseLiftCommand;
import org.firstinspires.ftc.teamcode.commands.lift.SetLiftHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class Drive extends CommandBasedTeleOp
{
    DriveTrainSubsystem driveTrain;
    LiftSubsystem liftSubsystem;
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    DucksSubsystem ducksSubsystem;
    LEDSubsystem ledSubsystem;
    // Drive Commands
    TankDriveCommand tankDriveCommand;
    FieldCentricArcadeDriveCommand arcadeDriveCommand;
    GeneralDriveLeftCommand driveLeftCommand;
    GeneralDriveRightCommand driveRightCommand;
    // Lift Commands
    RaiseLiftCommand raiseLiftCommand;
    SetLiftHeightCommand resetLiftCommand;
    // Arm commands
    RotateArmCommand rotateArmContinuouslyCommand;
    RotateArmToPositionCommand rotateArmToMiddleCommand;
    SetIntakeArmPositionCommand resetIntakeArmPositionCommand;
    // intake commands
    IntakeCommand intakeCommand;
    //Ducky command
    FancyDuckIndexCommand fancyDuckIndexCommand;
    //Multiple subsystem commands
    Command GoToIntakePositionCommand;
    SetRobotArmsPosition GoToScoringPositionCommand;
    SetRobotArmsPosition GoToSippingHubCommand;

    protected double getDriveSpeed() {
        if (gamepad1.left_trigger > 0)          return 0.5;
        else if (gamepad1.right_trigger > 0)    return 0.75;
        else                                    return 1;
    }

    private double getArmRotationPower() {
        double power = ((gamepad2.right_trigger - gamepad2.left_trigger > 0)? 1: (gamepad2.right_trigger - gamepad2.left_trigger < 0)? -1: 0)
                * ((gamepad2.dpad_left)? 0.2:0.5);
        if (armSubsystem.getAngle() > -80 && armSubsystem.getAngle() < 85)  return power;
        else if (power > 0 && armSubsystem.getAngle() < -80)                return power;
        else if (power < 0 && armSubsystem.getAngle() > 85)                 return power;
        else                                                                return 0;
    }

    private boolean canLowerArm() {
        return (-30 < armSubsystem.getAngle() && armSubsystem.getAngle() < 30) || liftSubsystem.getHeight() > 0.3;
    }

    public void saveArmsLocation() {
        if (Math.abs(armSubsystem.getAngle()) > 20)
            GoToScoringPositionCommand.setTarget(liftSubsystem.getHeight(), armSubsystem.getAngle(), armSubsystem.getVerticalPosition());
    }

    // TODO: git god
    double lastRumble = 0;

    @Override
    public void assign() {
        RobotUniversal.telemetryPacketUpdater = Drive.this::updateFtcDashboardTelemetry;
        driveTrain = new DriveTrainSubsystem();
        liftSubsystem = new LiftSubsystem();
        armSubsystem = new ArmSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        ducksSubsystem = new DucksSubsystem();
        ledSubsystem = new LEDSubsystem();

        driveTrain.setPose(RobotUniversal.endPosition);

        armSubsystem.setVerticalPosition(0.6);

        ledSubsystem.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);

        tankDriveCommand = new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y * getDriveSpeed(), () -> -gamepad1.right_stick_y * getDriveSpeed());
        driveLeftCommand = new GeneralDriveLeftCommand(driveTrain, () -> -gamepad1.left_stick_y);
        driveRightCommand = new GeneralDriveRightCommand(driveTrain, () -> -gamepad1.right_stick_y);

        raiseLiftCommand = new RaiseLiftCommand(liftSubsystem, () -> -gamepad2.left_stick_y);
        resetLiftCommand = new SetLiftHeightCommand(liftSubsystem,0, 0.7);

        rotateArmContinuouslyCommand = new RotateArmCommand(armSubsystem, this::getArmRotationPower);
        rotateArmToMiddleCommand = new RotateArmToPositionCommand(armSubsystem,0,1);
        resetIntakeArmPositionCommand = new SetIntakeArmPositionCommand(armSubsystem,0);

        intakeCommand = new IntakeCommand(intakeSubsystem, () -> gamepad2.right_stick_y);

        GoToIntakePositionCommand = new InstantCommand(
                () -> {saveArmsLocation(); intakeSubsystem.setDoorState(IntakeSubsystem.DoorState.Close); })
                .andThen(new SetRobotArmsPosition(armSubsystem, liftSubsystem, Constants.LiftConstants.lower_plate_height, 1, 0, 1, 0));
        GoToScoringPositionCommand = new SetRobotArmsPosition(armSubsystem, liftSubsystem, 0.20, 1, 50, 0.6, 0.5);

        GoToSippingHubCommand = new SetRobotArmsPosition(armSubsystem, liftSubsystem, 0.395, 1, 70, 1, 0.7);

        // DriveTrain commands
        driveTrain.setDefaultCommand(tankDriveCommand);
        gp1.dpad_left.whileHeld(driveLeftCommand);
        gp1.left_bumper.whileHeld(driveLeftCommand);
        gp1.dpad_right.whileHeld(driveRightCommand);
        new Trigger(() -> gamepad1.right_bumper && !gamepad1.a).whileActiveOnce(driveRightCommand);
        // Lift commands
        liftSubsystem.setDefaultCommand(raiseLiftCommand);
        // Arm command
        armSubsystem.setDefaultCommand(rotateArmContinuouslyCommand);

        gp2.left_bumper.whileHeld(() -> armSubsystem.setVerticalPosition(Math.min(armSubsystem.getVerticalPosition()+0.03, 1)));
        gp2.right_bumper.whileHeld(() -> armSubsystem.setVerticalPosition(Math.max(armSubsystem.getVerticalPosition()-0.03, 0)));

        gp2.b.whenActive(() -> armSubsystem.setVerticalPosition(1), armSubsystem);
        gp2.a.and(new Trigger(this::canLowerArm)).whenActive(() -> armSubsystem.setVerticalPosition(0), armSubsystem);
        gp2.x.whenActive(() -> armSubsystem.setVerticalPosition(0.4), armSubsystem);

        gp2.right_stick_button.whenPressed(GoToIntakePositionCommand);
        gp2.left_stick_button.whenPressed(GoToScoringPositionCommand.withInterrupt(() -> gamepad2.right_trigger > 0.2 || gamepad2.left_trigger > 0.2));
        gp2.dpad_up.whenPressed(GoToSippingHubCommand);
        // Intake commands
        intakeSubsystem.setDefaultCommand(intakeCommand);
        gp2.y.whenPressed(new InstantCommand(() -> intakeSubsystem.toggleDoor()).andThen(new IntakeCommand(intakeSubsystem, -0.2).withTimeout(0.2)));

        // Triggers
        // Indicate to drivers when a freight enters the robot
        new Trigger(intakeSubsystem::hasFreight).and(new Trigger(() -> armSubsystem.getVerticalPosition() == 0 && getRuntime() - lastRumble > 1))
                .whenActive(() -> {gamepad2.rumble(200); gamepad1.rumble(500); lastRumble=getRuntime();});

        // Telemetry
        // No need for anything but update in loop because use of suppliers
        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("dt(s)", this::dt);
        telemetry.addData("angle", armSubsystem::getAngle);
        telemetry.addData("lift height", liftSubsystem::getHeight);
        telemetry.addData("has freight", intakeSubsystem::hasFreight);
        telemetry.addData("lift height offset", LiftSubsystem.ticks2meters(liftSubsystem.getEncoderOffset()));
        telemetry.addData("LineColorSensorBrightness", driveTrain::getLineColorSensorBrightness);
        telemetry.addData("intake height", armSubsystem::getVerticalPosition);
        telemetry.addData("l pos", driveTrain::getLeftPosition);
        telemetry.addData("r pos", driveTrain::getRightPosition);
        telemetry.addData("left distance", driveTrain::getLeftDistance);
        telemetry.addData("right distance", driveTrain::getRightDistance);

        // green flash when a mineral enters the intake
        new Trigger(intakeSubsystem::hasFreight).whenActive(new CommandBase() {
            RevBlinkinLedDriver.BlinkinPattern pattern;
            @Override
            public void initialize() {
                pattern = ledSubsystem.getPattern();
                ledSubsystem.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            @Override
            public void end(boolean interrupted) {
                ledSubsystem.setPattern(pattern);
            }
        }.withTimeout(0.5));


        new Trigger(() -> getRuntime() > 75).whenActive(() -> ledSubsystem.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED));
        new Trigger(() -> getRuntime() > 110).whenActive(() -> ledSubsystem.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE));
    }

    public abstract void updateFtcDashboardTelemetry(TelemetryPacket packet);

    @Override
    public void start() {
        resetStartTime();
    }

    @Override
    public void end() {
        RobotUniversal.endPosition = driveTrain.getPoseEstimate();
    }
}