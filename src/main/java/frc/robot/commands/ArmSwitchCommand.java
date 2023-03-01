// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSwitchCommand extends ParallelCommandGroup {

	private static ArmSubsystem armSubsystem;
	private static LimelightSubsystem limelightSubsystem;

	/** Creates a new ArmSwitchCommand. */
	public ArmSwitchCommand() {

		armSubsystem = RobotContainer.armSubsystem;
		limelightSubsystem = RobotContainer.limelightSubsystem;

		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new InstantCommand(() -> armSubsystem.ToggleSide()),
				limelightSubsystem.FlipLimelight(armSubsystem.getIsFront()));
	}
}
