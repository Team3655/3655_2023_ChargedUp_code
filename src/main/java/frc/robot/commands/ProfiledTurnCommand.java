// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ProfiledTurnCommand extends ProfiledPIDCommand {
	/** Creates a new ProfiledTurnCommand. */
	public ProfiledTurnCommand(double targetAngleDegrees, DriveSubsystem driveSubsystem) {
		super(
				// The ProfiledPIDController used by the command
				new ProfiledPIDController(
						// The PID gains
						0.01,
						0,
						0,
						// The motion profile constraints
						new TrapezoidProfile.Constraints(100, 100)),
				// This should return the measurement
				driveSubsystem::getHeading360,
				// This should return the goal (can also be a constant)
				targetAngleDegrees,
				// This uses the output
				(output, setpoint) -> {
					driveSubsystem.drive(0, 0, -output);
				});
		// Use addRequirements() here to declare subsystem dependencies.
		// Configure additional PID options by calling `getController` here.
		// Set the controller to be continuous (because it is an angle controller)
		getController().enableContinuousInput(-180, 180);
		// Set the controller tolerance - the delta tolerance ensures the robot is
		// stationary at the setpoint before it is considered as having reached the
		// reference
		getController().setTolerance(
				AutoConstants.kTurnCommandToleranceDeg,
				AutoConstants.kTurnCommandRateToleranceDegPerS);

		addRequirements(driveSubsystem);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
