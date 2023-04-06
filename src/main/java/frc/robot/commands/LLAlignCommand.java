// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Mechanisms.Limelight;
import frc.robot.TractorToolbox.TractorParts.DoubleSmoother;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLAlignCommand extends CommandBase {

	private static LimelightSubsystem limelightSubsystem;
	private static DriveSubsystem driveSubsystem;

	Limelight limelight;

	PIDController StrafePIDController;
	PIDController DrivePIDController;

	DoubleSmoother driveOutputSmoother;
	DoubleSmoother strafeOutputSmoother;

	/** Creates a new LLTargetCommand. */
	public LLAlignCommand() {

		driveSubsystem = RobotContainer.driveSubsystem;
		limelightSubsystem = RobotContainer.limelightSubsystem;
		limelight = limelightSubsystem.limelight;

		StrafePIDController = new PIDController(
			LimelightConstants.kLLAlignStrafeGains.kP,
			LimelightConstants.kLLAlignStrafeGains.kI,
			LimelightConstants.kLLAlignStrafeGains.kD);
		StrafePIDController.setTolerance(0.2);

		DrivePIDController = new PIDController(
			LimelightConstants.kLLAlignDriveGains.kP,
			LimelightConstants.kLLAlignDriveGains.kI,
			LimelightConstants.kLLAlignDriveGains.kD);
		DrivePIDController.setTolerance(0.15);

		strafeOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignStrafeMotionSmoothing);
		driveOutputSmoother = new DoubleSmoother(LimelightConstants.kAlignDriveMotionSmoothing);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(limelightSubsystem, driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		limelight.setLedMode(3);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {


		if (limelight.hasValidTarget()) {

			SmartDashboard.putNumber("LL TX", limelight.getX());

			double strafePIDOutput = StrafePIDController.calculate(limelight.getX(), -0);
			double drivePIDOutput = DrivePIDController.calculate(limelight.getY(), -1);
			
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
		// limelight.setLedMode(1);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return LLTargetpidController.atSetpoint();
		return false;
	}
}
