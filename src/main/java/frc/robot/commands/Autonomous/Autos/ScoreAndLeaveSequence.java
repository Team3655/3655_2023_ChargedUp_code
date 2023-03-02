// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.commands.Autonomous.ScoreSequence;
import frc.robot.commands.Autonomous.TimedDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndLeaveSequence extends SequentialCommandGroup {

	private static DriveSubsystem driveSubsystem;
	private static ArmSubsystem armSubsystem;

	/** Creates a new ScoreAndLeaveSequence. */
	public ScoreAndLeaveSequence() {

		driveSubsystem = RobotContainer.driveSubsystem;
		armSubsystem = RobotContainer.armSubsystem;

		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new InstantCommand(() -> driveSubsystem.zeroHeading()),
				new InstantCommand(() -> armSubsystem.ToggleSide()),
				new ScoreSequence(ArmPoses.HIGH_SCORE),
				new InstantCommand(() -> Timer.delay(1)),
				new ArmPoseCommand(ArmPoses.TUCKED),
				new TimedDriveCommand(-.2, .2, 0, 500),
				new TimedDriveCommand(-.5, 0, 0, 1200));
	}
}