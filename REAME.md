# Robot Code for Megiddo Lions 18833 in FTC Freight-Frenzy season

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

### Our Autonomous loader

We use a custom language we developed for this purpose during this season.
It consist of two main parts, the first is the [interpreter itself](TeamCode/src/main/java/edu/megiddo/lions/).\
And the second part is the configuration we used in our [code](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/auto/).\
This also consist of a custom [trajectory loader](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/lib/auto/TrajectoryLoader.java) to load our trajectories from a json file.\
It's important to note that this part is not very stable right now and needs some farther work.

### The code itself

Just like any CommandBased FRC code our code is made of four main parts.\
The first one is the [Subsystems](#Subsystems),
each subsystem is responsible for one subsystem in our robot.
And all interactions with the hardware should be implemented in a Subsystem.

The second one is commands, for each functionality of our robot we use a command that defined that behavior.
Just like all access to a hardware device is through a Subsystem every usage of a subsystem other then initialization is through a command.
for example our [drive](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/drive/TankDriveCommand.java).

The third part is our [Constants](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java),
used to define every constants our robot uses.
The Constants class is divided for the subsystems so every subsystem has it's own constants class.

The forth part is the teleop themselves, kind of like the RobotContainer in FRC.
In every opmode we initialize the subsystems, put the telemetry data and assaying the commands to buttons and triggers.

## Subsystems

TODO: Write this section
