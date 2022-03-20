# Command Based stuff

This directory contains all stuff required to use WPILib in FTC other than the library itself.

## Classes

### Gp

This class is used to define a gamepad that is compatable with WPILib commands.\
For example to make a command run when driver presses on the A button we will write:

```java
gp1.a.whenPressed(command);
```

### Robot Universal

This class is used to store and define all of the robot globals variabls like the hardwareMap.

## OpModes

### CommandBasedTeleop

This opmode is used for telop opmodes, it has all the nice features teleop progreaming requires like GamePad classes (gp1, gp2).

### CommandBasedAuto

This class is used to define a CommandBased Autonomous.
This opmode loaded one command at start and run it.

### LinearOpModeWithCommands

This opmode is like the basic LinearOpMode but the CommandSqeduler is running in the background.
