// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.TractorToolbox.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LLTargetCubeCommand extends CommandBase {

	private static IntakeSubsystem intakeSubsystem;
	private static DriveSubsystem driveSubsystem;

	ProfiledPIDController LLTargetpidController;

	Timer timer;
	
	int failTimeMilis;

	/** Creates a new LLTargetCommand. */
	public LLTargetCubeCommand(int failTimeMilis) {

		driveSubsystem = RobotContainer.driveSubsystem;
		intakeSubsystem = RobotContainer.intakeSubsystem;

		timer = new Timer();

		this.failTimeMilis = failTimeMilis;

		LLTargetpidController = new ProfiledPIDController(
				LimelightConstants.kLLPuppyTurnGains.kP,
				LimelightConstants.kLLPuppyTurnGains.kI,
				LimelightConstants.kLLPuppyTurnGains.kD,
				new TrapezoidProfile.Constraints(40, 40));

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LimelightHelpers.setPipelineIndex("", LimelightConstants.kCubePipeline);
		LimelightHelpers.setLEDMode_ForceOn("");
		intakeSubsystem.setIntakeState(kIntakeStates.INTAKE);
		driveSubsystem.setFieldCentric(false);
		timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (LimelightHelpers.getTV("")) {

			double strafePIDOutput = LLTargetpidController.calculate(LimelightHelpers.getTX(""), 0);
			driveSubsystem.drive(.2, strafePIDOutput, 0);

		} else {
			driveSubsystem.drive(0, 0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		LimelightHelpers.setLEDMode_PipelineControl("");
		driveSubsystem.stopMotors();
		driveSubsystem.setFieldCentric(true);
	}	

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intakeSubsystem.getDistanceAsVolts() > 1.9 || Units.secondsToMilliseconds(timer.get()) > failTimeMilis;
	}
}
