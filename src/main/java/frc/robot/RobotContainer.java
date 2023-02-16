// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.JoystickUtils;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.commands.ArmSwitchCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final ArmSubsystem armSubsystem = new ArmSubsystem();
	private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);
	private final CommandGenericHID operatorController = new CommandGenericHID(1);

	Trigger select = driverController.back();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
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
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		new Trigger(exampleSubsystem::exampleCondition).whileTrue(new ExampleCommand(exampleSubsystem));

		// region Arm Commands
		// Schedule ArmPoseCommand when operator presses coresponding button.
		// scoring commands
		operatorController.button(1).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.LOW_SCORE));
		operatorController.button(2).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.MID_SCORE));
		operatorController.button(3).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.HIGH_SCORE));

		// intaking commands
		operatorController.button(6).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.LOW_INTAKE));
		operatorController.button(7).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.MID_INTAKE));
		operatorController.button(8).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.HIGH_INTAKE));

		// tuck arms
		operatorController.button(4).onTrue(new ArmPoseCommand(armSubsystem, ArmPoses.TUCKED));

		// Switches sides of the robot
		operatorController.button(9).onTrue(new ArmSwitchCommand(armSubsystem, limelightSubsystem));
		// endregion

		// Sucking is set to be the defaut state of the intake
		operatorController.button(10).whileTrue(intakeSubsystem.stopSucking());
		intakeSubsystem.setDefaultCommand(intakeSubsystem.Suck());

		// region Drive Commands
		driverController.start().onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));

		driverController.start().onTrue(new InstantCommand(() -> driveSubsystem.toggleFieldCentric()));

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new RunCommand(
						() -> driveSubsystem.drive(
								JoystickUtils.processJoystickInput(driverController.getLeftY()), // x axis
								JoystickUtils.processJoystickInput(driverController.getLeftX()), // y axis
								JoystickUtils.processJoystickInput(driverController.getLeftX()), // z axis
								driverController.getHID().getLeftStickButton()
						),
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
		PathPlannerTrajectory traj = PathPlanner.loadPath("Rotate", new PathConstraints(2, 3));

		return driveSubsystem.followTrajectoryCommand(traj, true);
	}
}