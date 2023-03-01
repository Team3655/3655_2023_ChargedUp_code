// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
import frc.robot.commands.ArmBumpCommand;
import frc.robot.commands.ArmSwitchCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.LLAlignCommand;
import frc.robot.commands.LLPuppydogCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.Autonomous.ScoreAndLeaveSequence;
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

	private final CommandJoystick driveJoystick = new CommandJoystick(OperatorConstants.kDriveJoystickPort);
	private final CommandJoystick turnJoystick = new CommandJoystick(OperatorConstants.kTurnJoystickPort);
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

		// region Paths
		PathPlannerTrajectory trajChargedUpTest = PathPlanner.loadPath("ChargedUpTest", new PathConstraints(3, 5));
		PathPlannerTrajectory trajNewPath = PathPlanner.loadPath("New Path", new PathConstraints(3.5, 2.0));
		PathPlannerTrajectory trajRotationTuningV2 = PathPlanner.loadPath("RotationTuningV2", new PathConstraints(2.5, 5));
		PathPlannerTrajectory trajTurn90 = PathPlanner.loadPath("90 turn", new PathConstraints(.1, .05));
		PathPlannerTrajectory traj2MetersX = PathPlanner.loadPath("2 metersX", new PathConstraints(1.0, .5));
		PathPlannerTrajectory traj2MetersY = PathPlanner.loadPath("2 metersY", new PathConstraints(1.0, .5));
		PathPlannerTrajectory trajOdometryHell = PathPlanner.loadPath("Odometry Hell", new PathConstraints(3.0, 2.0));
		// endregion

		autoChooser.setDefaultOption("2 metersX", driveSubsystem.followTrajectoryCommand(traj2MetersX, true));
		autoChooser.addOption("Odometry Hell", driveSubsystem.followTrajectoryCommand(trajOdometryHell, true));
		autoChooser.addOption("90 turn", driveSubsystem.followTrajectoryCommand(trajTurn90, true));
		autoChooser.addOption("2 metersY", driveSubsystem.followTrajectoryCommand(traj2MetersY, true));
		autoChooser.addOption("ChargedUpTest", driveSubsystem.followTrajectoryCommand(trajChargedUpTest, true));
		autoChooser.addOption("New Path", driveSubsystem.followTrajectoryCommand(trajNewPath, true));
		autoChooser.addOption("Rotation Tuning V2", driveSubsystem.followTrajectoryCommand(trajRotationTuningV2, true));
		autoChooser.addOption("Simple human player", new ScoreAndLeaveSequence());
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
		operatorController.button(9).onTrue(new ArmSwitchCommand());

		operatorController.button(11).onTrue(armSubsystem.toggleArmMotors());
		operatorController.button(13).onTrue(armSubsystem.zeroArms());

		operatorController.button(18).whileTrue(new ArmBumpCommand(+5, 0));
		operatorController.button(20).whileTrue(new ArmBumpCommand(-5, 0));

		operatorController.button(17).whileTrue(new ArmBumpCommand(0, +5));
		operatorController.button(19).whileTrue(new ArmBumpCommand(0, -5));
		// endregion

		// Sucking is set to be the defaut state of the intake
		operatorController.button(10).onTrue(intakeSubsystem.stopSucking()).onFalse(intakeSubsystem.startSucking());
		operatorController.button(5).onTrue(intakeSubsystem.toggleSideSucker());

		// region Targeting Commmands
		driveJoystick.button(3).whileTrue(new LLAlignCommand());
		driveJoystick.button(4).whileTrue(new TurnCommand(180));
		driveJoystick.button(5).whileTrue(new TurnCommand(180));
		programmerController.a().whileTrue(new LLAlignCommand());
		programmerController.b().whileTrue(new LLPuppydogCommand());
		programmerController.x().whileTrue(new TurnCommand(180));
		// endregion

		programmerController.y().whileTrue(new BalanceCommand());

		// region Drive Commands
		driveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		driveJoystick.button(12).onTrue(driveSubsystem.toggleFieldCentric());

		programmerController.button(8).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		programmerController.button(6).onTrue(driveSubsystem.toggleFieldCentric());

		driveJoystick.povUp().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(-0.05, 0, 0), driveSubsystem));
		driveJoystick.povDown().whileTrue(
				new RunCommand(() -> driveSubsystem.robotCentricDrive(0.05, 0, 0), driveSubsystem));

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				new RunCommand(
						() -> driveSubsystem.drive(
								JoystickUtils.processJoystickInput(driveJoystick.getY()) - JoystickUtils.processJoystickInput(programmerController.getLeftY()), // x axis
								JoystickUtils.processJoystickInput(driveJoystick.getX()) - JoystickUtils.processJoystickInput(programmerController.getLeftX()), // y axis
								JoystickUtils.processJoystickInput(turnJoystick.getX()) - JoystickUtils.processJoystickInput(programmerController.getRightX()), // rot axis
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