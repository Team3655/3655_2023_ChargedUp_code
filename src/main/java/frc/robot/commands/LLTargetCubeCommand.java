// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.Mechanisms.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LLTargetCubeCommand extends CommandBase {

	private static LimelightSubsystem limelightSubsystem;
	private static IntakeSubsystem intakeSubsystem;
	private static DriveSubsystem driveSubsystem;

	Limelight limelight;

	ProfiledPIDController LLTargetpidController;

	Timer timer;
	
	int failTimeMilis;

	/** Creates a new LLTargetCommand. */
	public LLTargetCubeCommand(int failTimeMilis) {

		driveSubsystem = RobotContainer.driveSubsystem;
		limelightSubsystem = RobotContainer.limelightSubsystem;
		intakeSubsystem = RobotContainer.intakeSubsystem;
		limelight = limelightSubsystem.limelight;

		timer = new Timer();

		this.failTimeMilis = failTimeMilis;

		LLTargetpidController = new ProfiledPIDController(
				LimelightConstants.kLLPuppyTurnGains.kP,
				LimelightConstants.kLLPuppyTurnGains.kI,
				LimelightConstants.kLLPuppyTurnGains.kD,
				new TrapezoidProfile.Constraints(40, 40));

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(limelightSubsystem, driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		limelight.setPipeline(LimelightConstants.kCubePipeline);
		limelight.setLedMode(3);
		intakeSubsystem.setIntakeState(kIntakeStates.INTAKE);
		driveSubsystem.setFieldCentric(false);
		timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (limelight.hasValidTarget()) {

			double strafePIDOutput = LLTargetpidController.calculate(limelight.getX(), 0);
			driveSubsystem.drive(.2, strafePIDOutput, 0);

		} else {
			driveSubsystem.drive(0, 0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		limelight.setLedMode(0);
		driveSubsystem.stopMotors();
		driveSubsystem.setFieldCentric(true);
	}	

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intakeSubsystem.getDistanceAsVolts() > 1.9 || Units.secondsToMilliseconds(timer.get()) > failTimeMilis;
	}
}
