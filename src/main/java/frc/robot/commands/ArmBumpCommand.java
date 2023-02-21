// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmBumpCommand extends InstantCommand {

	ArmSubsystem armSubsystem;
	double majorArmBump;
	double minorArmBump;

	public ArmBumpCommand(double majorArmBump, double minorArmBump, ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		this.majorArmBump = majorArmBump;
		this.minorArmBump = minorArmBump;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(armSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		armSubsystem.armStates.replace(ArmSubsystem.ArmPoses.DRIVER_CONTROL,
				new double[] {
						armSubsystem.getTargetTheta()[0] + majorArmBump,
						armSubsystem.getTargetTheta()[1] + minorArmBump });

		armSubsystem.ArmPoseCommand(ArmPoses.DRIVER_CONTROL);
	}

}
