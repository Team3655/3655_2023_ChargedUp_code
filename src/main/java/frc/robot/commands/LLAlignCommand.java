// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Objects.Limelight;
import frc.robot.Utils.Helpers.DoubleSmoother;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLAlignCommand extends CommandBase {

	LimelightSubsystem limelightSubsystem;
	DriveSubsystem driveSubsystem;

	Limelight limelight;

	PIDController LLStrafePIDController;
	PIDController LLDrivePIDController;

	DoubleSmoother driveOutput;
	DoubleSmoother strafeOutput;

	/** Creates a new LLTargetCommand. */
	public LLAlignCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.

		LLStrafePIDController = new PIDController(.03, 0.0001, 0.0005);
		LLStrafePIDController.setTolerance(.2);

		LLDrivePIDController = new PIDController(.03, 0.0001, 0.0005);
		LLDrivePIDController.setTolerance(0);

		this.limelightSubsystem = limelightSubsystem;
		this.driveSubsystem = driveSubsystem;
		limelight = this.limelightSubsystem.limelight;

		strafeOutput = new DoubleSmoother(.5);
		driveOutput = new DoubleSmoother(.5);

		addRequirements(limelightSubsystem, driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		driveSubsystem.setFieldCentric(false);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (limelight.hasValidTarget()) {
			double strafePIDOutput = LLStrafePIDController.calculate(limelight.getX(), 0);
			double drivePIDOutput = LLDrivePIDController.calculate(limelight.getY(), .4);

			driveSubsystem.drive(driveOutput.smoothInput(drivePIDOutput) + .02, strafeOutput.smoothInput(strafePIDOutput), 0);

		} else {
			driveSubsystem.drive(0, 0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveSubsystem.setFieldCentric(true);
		// limelight.disableTracking();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return LLTargetpidController.atSetpoint();
		return false;
	}
}
