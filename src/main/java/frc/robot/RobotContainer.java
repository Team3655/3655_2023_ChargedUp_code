// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.commands.ArmSwitchCommand;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Autos;


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
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final DashboardSubsystem m_dashboardSubsystem = new DashboardSubsystem(m_driveSubsystem);
	// private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);
	private final CommandGenericHID m_operatorController = new CommandGenericHID(1);

	Trigger select = m_driverController.back();
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
		new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

		select.onTrue(new InstantCommand(
			() -> m_driveSubsystem.zeroHeading()
		));

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed, cancelling on release.
		// m_driverController.a().onTrue(new ArmPoseCommand(m_armSubsystem,
		// ArmPoses.MID_SCORE));
		// m_driverController.b().onTrue(new ArmPoseCommand(m_armSubsystem,
		// ArmPoses.MID_INTAKE));
		// m_driverController.y().onTrue(new ArmPoseCommand(m_armSubsystem,
		// ArmPoses.LOW_SCORE));
		// m_driverController.x().onTrue(new ArmPoseCommand(m_armSubsystem,
		// ArmPoses.TUCKED));

		// Swerve Drive method is set as default for drive subsystem
		m_driveSubsystem.setDefaultCommand(
				new RunCommand(
						() -> m_driveSubsystem.drive(
								Math.pow(m_driverController.getLeftY(), 3) * DriveConstants.kMaxSpeedMetersPerSecond, // x axis
								Math.pow(m_driverController.getLeftX(), 3) * DriveConstants.kMaxSpeedMetersPerSecond, // y axis
								Math.pow(m_driverController.getRightX(), 3) * DriveConstants.kMaxRPM, // z axis
								true),
						m_driveSubsystem));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		PathPlannerTrajectory traj = PathPlanner.loadPath("TestPath", new PathConstraints(2, 3));

		return m_driveSubsystem.followTrajectoryCommand(traj, true);
	}
}
