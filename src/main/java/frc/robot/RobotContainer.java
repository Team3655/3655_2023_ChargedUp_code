// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.TractorToolbox.JoystickUtils;
import frc.robot.TractorToolbox.TractorParts.PathBuilder;
import frc.robot.commands.ArmSwitchCommand;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.LLAlignCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.Autonomous.BalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

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
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public static final ArmSubsystem armSubsystem = new ArmSubsystem();
	public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	public final PathBuilder autoBuilder = new PathBuilder();

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
		autoBuilder.addPath("1+1 Human Player");

		autoChooser.setDefaultOption("Event Test", autoBuilder.getPathCommand("Event Test"));
		autoChooser.addOption("1 Human Player", autoBuilder.getPathCommand("1 Human Player"));
		autoChooser.addOption("1 Wall", autoBuilder.getPathCommand("1 Wall"));
		autoChooser.addOption("1+1 Human Player", autoBuilder.getPathCommand("1+1 Human Player"));
		autoChooser.addOption("1 Charge Mobility",
				autoBuilder.getPathCommand("1 Charge Mobility").andThen(new BalanceCommand()));
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

		// region Arm Commands
		// Schedule ArmPoseCommand when operator presses coresponding button.
		// scoring commands
		operatorController.button(1).onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.LOW_SCORE));
		operatorController.button(2).onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.MID_SCORE));
		operatorController.button(3).onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.HIGH_SCORE));

		// intaking commands
		operatorController.button(6).onTrue(new FloorIntakeCommand());
		operatorController.button(7).onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.MID_INTAKE));
		operatorController.button(8).onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.HIGH_INTAKE));
		programmerController.b().onTrue(new FloorIntakeCommand());

		// tuck arms
		operatorController.button(4).onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.TUCKED));
		programmerController.y().onTrue(armSubsystem.SequencedArmPoseCommand(ArmPoses.TUCKED));

		// Switches sides of the robot
		operatorController.button(9).onTrue(new ArmSwitchCommand());

		operatorController.button(11).onTrue(armSubsystem.toggleArmMotors());
		operatorController.button(13).onTrue(armSubsystem.zeroArms());

		operatorController.button(22).onTrue(new InstantCommand(() -> armSubsystem.closeGriper()));
		operatorController.button(23).onTrue(new InstantCommand(() -> armSubsystem.openGriper()));

		// endregion

		// Sucking is set to be the defaut state of the intake
		operatorController.button(5).onTrue(intakeSubsystem.stopSuckingCommand()).onFalse(intakeSubsystem.startSuckingCommand());
		operatorController.button(10).onTrue(intakeSubsystem.stopSuckingCommand()).onFalse(intakeSubsystem.startSuckingCommand());
		operatorController.button(21).onTrue(intakeSubsystem.stopSuckingCommand());
		// operatorController.button(10).onTrue(intakeSubsystem.toggleDumpValve());

		// region Targeting Commmands
		driveJoystick.button(3).whileTrue(new LLAlignCommand());
		driveJoystick.button(4).whileTrue(new TurnCommand(180));
		driveJoystick.button(5).whileTrue(new TurnCommand(180));
		programmerController.a().whileTrue(new LLAlignCommand());
		programmerController.x().whileTrue(new TurnCommand(180));
		// endregion

		programmerController.y().whileTrue(new BalanceCommand()); // TODO: stress test balance
		operatorController.button(12).whileTrue(new BalanceCommand());

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
				new RunCommand(
						() -> driveSubsystem.drive(
								-JoystickUtils.processJoystickInput(driveJoystick.getY())
										- JoystickUtils.processJoystickInput(programmerController.getLeftY()), // x axis
								-JoystickUtils.processJoystickInput(driveJoystick.getX())
										- JoystickUtils.processJoystickInput(programmerController.getLeftX()), // y axis
								-JoystickUtils.processJoystickInput(turnJoystick.getX())
										- JoystickUtils.processJoystickInput(programmerController.getRightX()), // rot
																												// axis
								driveJoystick.getHID().getRawButton(1), // turbo boolean
								driveJoystick.getHID().getRawButton(2)), // sneak boolean
						driveSubsystem));
		// endregion
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}
}