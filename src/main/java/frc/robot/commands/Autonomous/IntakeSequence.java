// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends SequentialCommandGroup {

	public static IntakeSubsystem intakeSubsystem;

	/** Creates a new ScoreCommand. */
	public IntakeSequence() {

		intakeSubsystem = RobotContainer.intakeSubsystem;

		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new InstantCommand(() -> intakeSubsystem.startSucking()),
				new ArmPoseCommand(ArmPoses.LOW_INTAKE));
	}
}
