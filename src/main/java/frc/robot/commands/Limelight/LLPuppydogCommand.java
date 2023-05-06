// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.LimelightConstants;
import frc.robot.RobotContainer;
import frc.robot.TractorToolbox.LimelightHelpers;
import frc.robot.TractorToolbox.TractorParts.DoubleSmoother;
import frc.robot.subsystems.DriveSubsystem;

public class LLPuppydogCommand extends CommandBase {

	private static DriveSubsystem driveSubsystem;

	ProfiledPIDController LLTargetpidController;
	PIDController LLDrivepidController;

	DoubleSmoother driveOutputSmoother;
	DoubleSmoother turnOutputSmoother;

	/** Creates a new LLTargetCommand. */
	public LLPuppydogCommand() {
		
		driveSubsystem = RobotContainer.driveSubsystem;

		LLTargetpidController = new ProfiledPIDController(
				LimelightConstants.kLLPuppyTurnGains.kP,
				LimelightConstants.kLLPuppyTurnGains.kI,
				LimelightConstants.kLLPuppyTurnGains.kD,
				new TrapezoidProfile.Constraints(40, 40));

		LLDrivepidController = new PIDController(
				LimelightConstants.kLLPuppyDriveGains.kP,
				LimelightConstants.kLLPuppyDriveGains.kI,
				LimelightConstants.kLLPuppyDriveGains.kD);

		turnOutputSmoother = new DoubleSmoother(LimelightConstants.kPuppyTurnMotionSmoothing);
		driveOutputSmoother = new DoubleSmoother(LimelightConstants.kPuppyDriveMotionSmoothing);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LimelightHelpers.setLEDMode_ForceOn("");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (LimelightHelpers.getTV("")) {
			double turnPIDOutput = LLTargetpidController.calculate(LimelightHelpers.getTX(""), 0);
			double drivePIDOutput = LLDrivepidController.calculate(LimelightHelpers.getTY(""), .5);

			double turnOutput = turnOutputSmoother.smoothInput(turnPIDOutput);
			double driveOutput = driveOutputSmoother.smoothInput(drivePIDOutput);

			driveSubsystem.robotCentricDrive(driveOutput, 0, -turnOutput);
		} else {
			driveSubsystem.robotCentricDrive(0, 0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		LimelightHelpers.setLEDMode_PipelineControl("");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return LLTargetpidController.atSetpoint();
		return false;
	}
}
