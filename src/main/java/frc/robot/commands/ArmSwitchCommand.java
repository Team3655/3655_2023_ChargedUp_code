// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;

/** A command used to tell the arms to run to a set of angles */
public class ArmSwitchCommand extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private ArmSubsystem m_armSubsystem;
	private IntakeSubsystem m_intakeSubsystem;

	/**
	 * Creates a new ArmPoseCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ArmSwitchCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
		m_armSubsystem = armSubsystem;
		m_intakeSubsystem = intakeSubsystem;

		// Only allows the robot to switch if it's holding a cude (safetey issue)
		if (m_intakeSubsystem.getHasCube()) {

			addCommands(
					// Tuck the robot first (set to not be interruptable)
					new ArmPoseCommand(m_armSubsystem, ArmPoses.TUCKED)
							.withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
					// sets the new dominant side of the robot
					m_armSubsystem.ToggleSide(),
					// goes to prev position but now on the other side of the bot
					new ArmPoseCommand(m_armSubsystem, m_armSubsystem.getPrevArmState()));

		} else {
			// Notify drivers if the robot cannot be a switch
			System.out.println("WARNING: cannot traverse robot without holding CUBE");
		}

	}

}
