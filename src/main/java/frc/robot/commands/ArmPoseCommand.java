// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command used to tell the arms to run to a set of angles */
public class ArmPoseCommand extends CommandBase {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private ArmSubsystem m_armSubsystem;
	private ArmSubsystem.ArmPoses m_armState;
	private ArmSubsystem.ArmPoses m_prevArmState;

	/**
	 * Creates a new ArmPoseCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ArmPoseCommand(ArmSubsystem armSubsystem, ArmSubsystem.ArmPoses armState) {
		m_armSubsystem = armSubsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(armSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_prevArmState = m_armSubsystem.getArmState();
		m_armSubsystem.setArmState(ArmSubsystem.ArmPoses.TUCKED);
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
		if (m_armSubsystem.getAtTarget(10)) {
			return true;
		}
		return false;
	}

}
