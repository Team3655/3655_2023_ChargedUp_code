// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.TractorToolbox.LimelightHelpers;
import frc.robot.TractorToolbox.TractorParts.DoubleSmoother;
import frc.robot.subsystems.DriveSubsystem;

public class LLAlignCommand extends CommandBase {

	private static DriveSubsystem driveSubsystem;

	PIDController StrafePIDController;
	PIDController DrivePIDController;

	DoubleSmoother driveOutputSmoother;
	DoubleSmoother strafeOutputSmoother;

	boolean targetTags;

	/** Creates a new LLTargetCommand. */
	public LLAlignCommand(boolean targetTags) {

		driveSubsystem = RobotContainer.driveSubsystem;

		this.targetTags = targetTags;

		StrafePIDController = new PIDController(
				LimelightConstants.kLLAlignStrafeGains.kP,
				LimelightConstants.kLLAlignStrafeGains.kI,
				LimelightConstants.kLLAlignStrafeGains.kD);
		StrafePIDController.setTolerance(0.25);

		DrivePIDController = new PIDController(
				LimelightConstants.kLLAlignDriveGains.kP,
				LimelightConstants.kLLAlignDriveGains.kI,
				LimelightConstants.kLLAlignDriveGains.kD);
		DrivePIDController.setTolerance(0.5);

		strafeOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignStrafeMotionSmoothing);
		driveOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignDriveMotionSmoothing);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		if (targetTags) {
			LimelightHelpers.setPipelineIndex("", LimelightConstants.kApriltagPipeline);

		} else {
			LimelightHelpers.setPipelineIndex("", LimelightConstants.kReflectivePipeline);

		}

		LimelightHelpers.setLEDMode_ForceOn("");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (LimelightHelpers.getTV("")) {

			SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""));

			double strafePIDOutput = StrafePIDController.calculate(LimelightHelpers.getTX(""), 0);
			double drivePIDOutput = DrivePIDController.calculate(LimelightHelpers.getTY(""), 0);

			double strafeOutput = strafeOutputSmoother.smoothInput(strafePIDOutput);
			double driveOutput = driveOutputSmoother.smoothInput(drivePIDOutput);

			driveSubsystem.drive(driveOutput, -strafeOutput, 0);

		} else {
			driveSubsystem.drive(0, 0, 0);
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
		return DrivePIDController.atSetpoint() && StrafePIDController.atSetpoint();
	}
}
