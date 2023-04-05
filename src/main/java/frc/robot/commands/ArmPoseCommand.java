// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmPoseCommand extends CommandBase {

	private static ArmSubsystem armSubsystem;
	private static IntakeSubsystem intakeSubsystem;
	private kArmPoses armPose;

	/** Creates a new ArmPoseCommand. */
	public ArmPoseCommand(kArmPoses armPose) {
		armSubsystem = RobotContainer.armSubsystem;
		intakeSubsystem = RobotContainer.intakeSubsystem;
		this.armPose = armPose;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(armSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		armSubsystem.setSequencedArmState(armPose);
		intakeSubsystem.updateIntakeFromArmPose(armPose);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return armSubsystem.getAtTarget(8);
	}

}
