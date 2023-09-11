// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.TractorToolbox.TractorParts.PathBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.Limelight.LLAlignCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

	public final static PathBuilder autoBuilder = new PathBuilder();

	private final CommandJoystick driveJoystick = new CommandJoystick(
			OperatorConstants.kDriveJoystickPort);
	private final CommandJoystick turnJoystick = new CommandJoystick(
			OperatorConstants.kTurnJoystickPort);
	private final CommandGenericHID operatorController = new CommandGenericHID(
			OperatorConstants.kOperatorControllerPort);
	private final CommandXboxController programmerController = new CommandXboxController(
			OperatorConstants.kProgrammerControllerPort);

	private SendableChooser<Command> autoChooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// region Def Auto
		Shuffleboard.getTab("Autonomous").add(autoChooser);

		// autoBuilder.populatePathMap(); // TODO: fix auto builder
		autoBuilder.addPath("Event Test");
		autoBuilder.addPath("1 Human Player");
		autoBuilder.addPath("1 Wall");
		autoBuilder.addPath("1 Charge Mobility");
		autoBuilder.addPath("1 Charge");
		autoBuilder.addPath("1+1 Human Player");
		autoBuilder.addPath("1+2 Human Player");
		autoBuilder.addPath("1+1.5 Human Player");
		autoBuilder.addPath("Square");
		autoBuilder.addPath("Cube Target Test");
		autoBuilder.addPath("1.5 Charge Mobility");


		autoChooser.addOption("Event Test", autoBuilder.getPathCommand("Event Test"));
		autoChooser.addOption("Square", autoBuilder.getPathCommand("Square"));
		autoChooser.addOption("Target Cube Test", autoBuilder.getPathCommand("Cube Target Test"));
		autoChooser.addOption("Odometry Hell", autoBuilder.getPathCommand("Odometry Hell"));

		autoChooser.addOption("1 Human Player", autoBuilder.getPathCommand("1 Human Player"));
		autoChooser.addOption("1 Wall", autoBuilder.getPathCommand("1 Wall"));
		autoChooser.addOption("1+1 Human Player", autoBuilder.getPathCommand("1+1 Human Player"));
		autoChooser.addOption("1+2 Human Player", autoBuilder.getPathCommand("1+2 Human Player"));
		autoChooser.addOption("1+1.5 Human Player", autoBuilder.getPathCommand("1+1.5 Human Player"));
		// endregion
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {


		operatorController.button(1).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));

		// region Targeting Commmands
		driveJoystick.button(3).whileTrue(new LLAlignCommand(false));
		driveJoystick.button(4).whileTrue(new LLAlignCommand(true));
		programmerController.a().whileTrue(new LLAlignCommand(false));
		programmerController.x().whileTrue(new TurnCommand(180));
		// endregion

		// region Drive Commands
		driveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		driveJoystick.button(12).onTrue(driveSubsystem.toggleFieldCentric());

		programmerController.button(8).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		programmerController.button(6).onTrue(driveSubsystem.toggleFieldCentric());

		driveJoystick.povUp().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(0.05, 0, 0), driveSubsystem));
		driveJoystick.povDown().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(-0.05, 0, 0), driveSubsystem));

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new TeleopDriveCommand(
						() -> -driveJoystick.getY() -programmerController.getLeftY(),
						() -> -driveJoystick.getX() -programmerController.getLeftX(),
						() -> -turnJoystick.getX() -programmerController.getRightX(),
						() -> driveJoystick.getHID().getRawButton(1)
								|| programmerController.rightBumper().getAsBoolean(),
						() -> driveJoystick.getHID().getRawButton(2)
								|| programmerController.rightBumper().getAsBoolean()));
		// endregion
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		driveSubsystem.setHeading(180);
		Timer.delay(0.1);
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}
}