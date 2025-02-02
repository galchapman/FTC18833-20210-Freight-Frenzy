# Robot Code for Megiddo Lions 18833 in FTC Freight-Frenzy season

## Our Robot

![No image found](robot.jpeg)

## Project Structure

### WPILib

We used [WPILib](https://docs.wpilib.org/en/stable/index.html) a library developed to control FRC robots.
We use it mainly for convince. Writing our code in term of commands who get scheduled and run make the code much cleaner.
This also allows some of the stuff I will go over in a bit like loading our autonomous from a file.\
We ported the code our self last year from [their source](https://github.com/wpilibsuite/allwpilib).
All of the code ander [edu.wpi](TeamCode/src/main/java/edu/wpi/) is theres.
The code for our teleop wrappers in [org.commandftc](TeamCode/src/main/java/org/commandftc/).

### Roadrunner

We use [RoadRunner](https://learnroadrunner.com/) to control our autonomous drive, and to calculate our position using dead wheels.

### [Our Autonomous loader](#AutonomousLoader)

We use a custom language we developed for this purpose during this season.
It consist of two main parts, the first is the [interpreter itself](TeamCode/src/main/java/edu/megiddo/lions/).\
And the second part is the configuration we used in our [code](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/auto/).\
This also consist of a custom [trajectory loader](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/auto/TrajectoryLoader.java) to load our trajectories from a json file.\
It's important to note that this part is not very stable right now and needs some farther work.

### The code itself

Just like any CommandBased FRC code our code is made of four main parts.\
The first one is the [Subsystems](#Subsystems),
each subsystem is responsible for controlling the robot hardware, And is used as a layer between the Commands and the hardware.

The second one is commands, for each functionality of our robot we use a command that defines it's behavior.
Just like all access to a hardware devices is through a Subsystem every usage of a subsystem other then initialization is through a command.
for example our [drive command](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/drive/TankDriveCommand.java).

The third part is our [Constants](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java),
used to define every constants our robot uses.
The Constants class is divided for the subsystems so every subsystem has it's own constants class.

The forth part is the opmodes themselves, kind of like the RobotContainer in FRC.
In every opmode we initialize the subsystems, put the telemetry data and assassin the commands to buttons and triggers.

## Subsystems

### DriveTrain

Our drive train subsystem includes four DC Motors, and one more motor for the encoder slot.\
It's also includes a Servo to lower the horizontal optometry wheel,
two Distance sensor and one Color sensor. That we planed to determine our robot position during the teleop faze.
This class also includes our odometry system.

This includes everything that makes the robot drive like the trajectory follower.

Note that this is the only subsystem in our robot that could run on it's own [thread](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/drive/RoadRunnerThread.java). That's is why many functions have the synchronized keyword.

### Lift

This subsystem control our lifter.
It's includes a DcMotor with an encoder and a limit switch to reset the lifter height.

### Arm

The subsystem includes the motor and servos in our arm.
It controls the motor that control the arm angle.
The arm could be control by two main ways.
The first is by power, and the other is by angle.
The subsystem also includes the servos that control the arm height.

### Intake

This class controls our intake, it consists of a motor, servo and a distance sensor.
The servo controls the door in our intake.
The distance sensor is used to detect when a friegth is in our intake.

### Ducks

This subsystem simply indexed the ducks. That's about it.

### [Vision](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/VisionSubsystem.java)

This class includes all of our vision stuff.
Like detection of the Team element at the start of the game.
We used [GRIP](https://wpiroboticsprojects.github.io/GRIP/#/) to create the pipeline.
Feel free to ask me about any part of this code.

### LEDs

Controls the LEDs.

## Commands

Most commands are very simple so I won't list them here, but I will include a few notable exceptions.

### The Trajectory commands

Those commands are crucial to anyone who's trying to use RoadRunner and WPILib.
They are used to interact with Road Runner like any other command.

### The general drive commands

Because our driver is used to tank drive we control the robot with a mix of tank and arcade drive.
So we could take advantage of both drive technics.

### The Thread

We have decided to run RoadRunner on it's own thread.
So we have a command to control the thread.

### Fancy Duck Index

Index the duck using fancy stuff like acceleration.

## Triggers

### Time Triggers

We use triggers to change the LED color during a match.

### Intake Trigger

We use an trigger to change the LED color to green and rumble the gamepad when a freight is in our intake.

## AutonomousLoader

We created a custom language for our autonomous, and [used it](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/) For our autonomous.
To install them we simply pushed them to the robot controller.
Each auto was written in two files, the first was .auto and included all of the logic of the language.
The other part was .json and it contained all of our trajectories for that auto.

We are planning to publish it for the next FTC Season and we will provide a link to the repository here.
