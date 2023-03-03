// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Mechanisms.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLTargetCommand extends CommandBase {

	private static LimelightSubsystem limelightSubsystem;
	private static DriveSubsystem driveSubsystem;

	Limelight limelight;

	PIDController LLTargetpidController;

	/** Creates a new LLTargetCommand. */
	public LLTargetCommand() {

		driveSubsystem = RobotContainer.driveSubsystem;
		limelightSubsystem = RobotContainer.limelightSubsystem;
		limelight = limelightSubsystem.limelight;

		LLTargetpidController = new PIDController(
				LimelightConstants.LLTargetGains.kP,
				LimelightConstants.LLTargetGains.kI,
				LimelightConstants.LLTargetGains.kD);
		LLTargetpidController.setTolerance(0);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(limelightSubsystem, driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double turnOutput = LLTargetpidController.calculate(limelight.getX(), 0);
		driveSubsystem.drive(0, 0, turnOutput);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return LLTargetpidController.atSetpoint();
	}
}
