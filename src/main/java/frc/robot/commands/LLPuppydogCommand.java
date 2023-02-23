// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Objects.Limelight;
import frc.robot.Utils.Helpers.DoubleSmoother;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLPuppydogCommand extends CommandBase {

	LimelightSubsystem limelightSubsystem;
	DriveSubsystem driveSubsystem;

	Limelight limelight;

	ProfiledPIDController LLTargetpidController;
	PIDController LLDrivepidController;

	DoubleSmoother driveOutput;
	DoubleSmoother turnOutput;

	/** Creates a new LLTargetCommand. */
	public LLPuppydogCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.

		LLTargetpidController = new ProfiledPIDController(LimelightConstants.LLP, LimelightConstants.LLI,
				LimelightConstants.LLD, new TrapezoidProfile.Constraints(40, 40));
		LLTargetpidController.setTolerance(0	);

		LLDrivepidController = new PIDController(.5, 0.0001, 0.0005);
		LLDrivepidController.setTolerance(0);

		this.limelightSubsystem = limelightSubsystem;
		this.driveSubsystem = driveSubsystem;
		limelight = this.limelightSubsystem.limelight;

		turnOutput = new DoubleSmoother(.4);
		driveOutput = new DoubleSmoother(.5);

		addRequirements(limelightSubsystem, driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		driveSubsystem.setFieldCentric(false);
		//limelight.enableTracking();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (limelight.hasValidTarget()) {
			double turnPIDOutput = LLTargetpidController.calculate(limelight.getX(), 0);
			double drivePIDOutput = LLDrivepidController.calculate(limelight.getArea(), .5);

			driveSubsystem.drive(driveOutput.smoothInput(drivePIDOutput), 0, -turnOutput.smoothInput(turnPIDOutput));
		} else {
			driveSubsystem.drive(0, 0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveSubsystem.setFieldCentric(true);
		//limelight.disableTracking();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return LLTargetpidController.atSetpoint();
		return false;
	}
}
