// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.kArmPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmBumpCommand extends InstantCommand {

	private static ArmSubsystem armSubsystem;
	private double majorArmBump;
	private double minorArmBump;

	public ArmBumpCommand(double majorArmBump, double minorArmBump) {

		armSubsystem = RobotContainer.armSubsystem;

		this.majorArmBump = majorArmBump;
		this.minorArmBump = minorArmBump;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(armSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		armSubsystem.armStates.replace(kArmPoses.DRIVER_CONTROL,
				new double[] {
						armSubsystem.getTargetTheta()[0] + majorArmBump,
						armSubsystem.getTargetTheta()[1] + minorArmBump });

		armSubsystem.UnsequencedArmPoseCommand(kArmPoses.DRIVER_CONTROL);
	}

}
