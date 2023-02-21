// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;

import frc.robot.commands.ExampleCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.JoystickUtils;
import frc.robot.commands.ArmBumpCommand;
import frc.robot.commands.ArmSwitchCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;
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
	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final ArmSubsystem armSubsystem = new ArmSubsystem();
	private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverController = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);

	private final CommandJoystick DriveJoystick = new CommandJoystick(0);
	private final CommandJoystick TurnJoystick = new CommandJoystick(1);

	private final CommandGenericHID operatorController = new CommandGenericHID(2);

	private SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		PathPlannerServer.startServer(5811);

		Shuffleboard.getTab("Autonomous").add(autoChooser);

		PathPlannerTrajectory trajTesting = PathPlanner.generatePath(
				new PathConstraints(1, 2),
				new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position,
																												// heading
				new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(0)),
				new PathPoint(new Translation2d(2, 1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
				new PathPoint(new Translation2d(0, 1), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
				new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position,
																												// heading
		);

		PathPlannerTrajectory xtraj = PathPlanner.generatePath(
				new PathConstraints(3, 4),
				new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
				new PathPoint(new Translation2d(4, 0), Rotation2d.fromDegrees(0)));

		PathPlannerTrajectory ytraj = PathPlanner.generatePath(
				new PathConstraints(3, 4),
				new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(-90)),
				new PathPoint(new Translation2d(0, -4), Rotation2d.fromDegrees(-90)));

		PathPlannerTrajectory trajRotationTuning = PathPlanner.generatePath(
				new PathConstraints(1, 2),
				new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
				new PathPoint(new Translation2d(2, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)));

		PathPlannerTrajectory trajUPath = PathPlanner.loadPath("Upath", new PathConstraints(2, 3));
		PathPlannerTrajectory trajChargedUpTest = PathPlanner.loadPath("ChargedUpTest", new PathConstraints(3, 5));
		PathPlannerTrajectory trajNewPath = PathPlanner.loadPath("New Path", new PathConstraints(3, 4));
		PathPlannerTrajectory trajRotationTuningV2 = PathPlanner.loadPath("RotationTuningV2", new PathConstraints(2.5, 5));


		autoChooser.addOption("UPath", trajUPath);
		autoChooser.addOption("Testing", trajTesting);
		autoChooser.addOption("ChargedUpTest", trajChargedUpTest);
		autoChooser.addOption("x Traj", xtraj);
		autoChooser.addOption("y Traj", ytraj);
		autoChooser.addOption("Rotation Tuning", trajRotationTuning);
		autoChooser.addOption("New Path", trajNewPath);
		autoChooser.addOption("Rotation Tuning V2", trajRotationTuningV2);
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
		operatorController.button(1).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.LOW_SCORE));
		operatorController.button(2).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.MID_SCORE));
		operatorController.button(3).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.HIGH_SCORE));

		// intaking commands
		operatorController.button(6).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.LOW_INTAKE));
		operatorController.button(7).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.MID_INTAKE));
		operatorController.button(8).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.HIGH_INTAKE));

		// tuck arms
		operatorController.button(4).onTrue(armSubsystem.ArmPoseCommand(ArmPoses.TUCKED));

		// Switches sides of the robot
		operatorController.button(9).onTrue(new ArmSwitchCommand(armSubsystem, limelightSubsystem));

		operatorController.button(11).onTrue(armSubsystem.toggleArmMotors());
		operatorController.button(13).onTrue(armSubsystem.zeroArms());

		operatorController.button(18).whileTrue(new ArmBumpCommand(1, 0, armSubsystem));
		// endregion

		// Sucking is set to be the defaut state of the intake
		operatorController.button(10).onTrue(intakeSubsystem.stopSucking()).onFalse(intakeSubsystem.startSucking());
		operatorController.button(5).onTrue(intakeSubsystem.toggleSideSucker());

		// region Drive Commands
		DriveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		DriveJoystick.button(12).onTrue(new InstantCommand(() -> driveSubsystem.toggleFieldCentric()));
		

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new RunCommand(
						() -> driveSubsystem.drive(
								JoystickUtils.processJoystickInput(DriveJoystick.getRawAxis(1)),     // x axis
								JoystickUtils.processJoystickInput(DriveJoystick.getRawAxis(0)),     // y axis
								JoystickUtils.processJoystickInput(TurnJoystick.getRawAxis(0)),      // rot axis
								DriveJoystick.getHID().getRawButton(1),  // turbo boolean
								DriveJoystick.getHID().getRawButton(2)), // sneak boolean
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

		return driveSubsystem.followTrajectoryCommand(autoChooser.getSelected(), true);
	}
}