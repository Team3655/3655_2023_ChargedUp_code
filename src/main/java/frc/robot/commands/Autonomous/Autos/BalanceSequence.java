// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.commands.ArmSwitchCommand;
import frc.robot.commands.Autonomous.ScoreSequence;
import frc.robot.commands.Autonomous.TimedDriveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceSequence extends SequentialCommandGroup {
	/** Creates a new BalanceSequence. */
	public BalanceSequence() {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new ArmSwitchCommand(),
				new ScoreSequence(ArmPoses.HIGH_SCORE),
				new ArmSwitchCommand(),
				new TimedDriveCommand(-.5, 0, 0, 1000)
				// new BalanceSequence()
				);
	}
}