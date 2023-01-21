
# FRC-2023

Team 3655's 2023 FRC robot code for [tbd](*link to robot*). tbd's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2022-Public.ipr` file with IntelliJ

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2022-Public` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Code Highlights
* Field-Centric Swerve Drive

    Standard [field-centric control of a swerve drivebase](src/main/java/com/team254/frc2022/subsystems/Drive.java) using odometry, encoders and gyro along with [Swerve Setpoint Generation](src/main/java/com/team254/lib/swerve/SwerveSetpointGenerator.java) to impose Kinematic constraints on the drivebase for more controlled movements to reduce wheel-slip and improve Shoot-On-The-Move.

* Superstructure with Two-Sided Intake, Automatic Wrong Ball Rejection, Super Eject, Automatic Turret Tracking

    The robot uses a [state machine](src/main/java/com/team254/frc2022/subsystems/Serializer.java) to control the intakes and serializers on both sides of the robot. It uses banner sensors to determine the locations of the ball in the robot and [REV Color Sensors wired to a Teensy](Teensy4-RevSensor/Teensy4-RevSensor.ino) to determine ball color. After detecting a wrong color ball, the robot determines the [eject setpoints based on its position and orientation on the field](src/main/java/com/team254/frc2022/subsystems/Superstructure.java#L369-502). Throughout the match, the robot always follows the vision target with the turret using [Motion Profiling](src/main/java/com/team254/lib/drivers/ServoMotorSubsystem.java#L463-475).

* Shooting on the Move
    
    The robot uses a [Sin Map](src/main/java/com/team254/frc2022/subsystems/Superstructure.java#L354-361) (Utilizing Trigonometry for both Shooter RPM and Hood Angle). We break Shooter RPM into a horizontal and vertical component and use a linear regression to map the robot's distance from goal to a Shooter RPM. To compensate for motion, the robot calculates a [feedforward based on its tangential and radial velocity about the goal](src/main/java/com/team254/frc2022/shooting/ShootingUtil.java#L26-57).

* Feedforward + Proportional Feedback Controller for Autonomous Path Following

    The robot generates pre-computed trajectories using a [Quintic Hermite Spline](https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/spline/SplineGenerator.java). To follow trajectories, the robot is first commanded the [precomputed velocity at the current timestamp](src/main/java/com/team254/frc2022/planners/DriveMotionPlanner.java#L302-326). It then computes the longitudinal error along the path and uses a [proportional controller](src/main/java/com/team254/frc2022/planners/DriveMotionPlanner.java#L187-200) to reduce it. The robot time-parametrizes heading based on the total timing of the trajectory and uses a separate proportional controller to reach heading setpoints.
    
* Fully Automated Climb

    The robot uses a [state machine for the endgame climb](src/main/java/com/team254/frc2022/subsystems/Climber.java) to move mechanisms in a controlled fashion. Transitions between states are determined using encoder positions, time in state, and gyro measurements.

## Package Functions
- [`com.team254.frc2022`](src/main/java/com/team254/frc2022)

    Contains the robot's central functions and holds a class with all numerical constants used throughout the code (see [`Constants.java`](src/main/java/com/team254/frc2022/Constants.java)). For example, the [`Robot`](src/main/java/com/team254/frc2022/Robot.java) class controls all routines depending on the robot mode. In addition, the [`RobotState`](src/main/java/com/team254/frc2022/RobotState.java) class keeps track of the current position of the robot's various frames of reference.
	
## Variable Naming Conventions
- k*** (i.e. `kDriveWheelbaseMeters`): Final constants, especially those found in the [`Constants.java`](src/main/java/com/team254/frc2022/Constants.java) file
- m*** (i.e. `mPathFollower`): Private instance variables