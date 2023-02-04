// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A command used to tell the arms to run to a set of angles */
public class ArmSwitchCommand extends SequentialCommandGroup {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private ArmSubsystem m_armSubsystem;

	/**
	 * Creates a new ArmPoseCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public ArmSwitchCommand(ArmSubsystem armSubsystem) {
		addCommands(
				new ArmPoseCommand(m_armSubsystem, ArmPoses.TUCKED)
						.withInterruptBehavior(InterruptionBehavior.kCancelIncoming),

				m_armSubsystem.toggleSide(),
				new ArmPoseCommand(m_armSubsystem, m_armSubsystem.getPrevArmState()));
	}

}
