// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndLeaveSequence extends SequentialCommandGroup {
	/** Creates a new ScoreAndLeaveSequence. */
	public ScoreAndLeaveSequence(
			ArmSubsystem armSubsystem,
			IntakeSubsystem intakeSubsystem,
			LimelightSubsystem limelightSubsystem,
			DriveSubsystem driveSubsystem) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new InstantCommand(() -> driveSubsystem.zeroHeading()),
				new InstantCommand(() -> armSubsystem.ToggleSide()),
				new ScoreSequence(ArmPoses.HIGH_SCORE, armSubsystem, intakeSubsystem, limelightSubsystem),
				new InstantCommand(() -> Timer.delay(1)),
				new ArmPoseCommand(ArmPoses.TUCKED, armSubsystem),
				new TimedDriveCommand(-.2, .2, 0, 500, driveSubsystem),
				new TimedDriveCommand(-.5, 0, 0, 1200, driveSubsystem));
	}
}