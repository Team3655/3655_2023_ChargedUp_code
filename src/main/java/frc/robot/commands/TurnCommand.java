// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnCommand extends CommandBase {

	private static DriveSubsystem driveSubsystem;
	private static PIDController turnPIDController;
	private double targetAngleDegrees;

	/** Creates a new ProfiledTurnCommand. */
	public TurnCommand(double targetAngleDegrees) {

		driveSubsystem = RobotContainer.driveSubsystem;
		this.targetAngleDegrees = targetAngleDegrees;

		turnPIDController = new PIDController(
				// The PID gains
				AutoConstants.kTurnCommandGains.kP,
				AutoConstants.kTurnCommandGains.kI,
				AutoConstants.kTurnCommandGains.kD);

		turnPIDController.enableContinuousInput(-180, 180);

		turnPIDController.setTolerance(
				AutoConstants.kTurnCommandToleranceDeg,
				AutoConstants.kTurnCommandRateToleranceDegPerS);

		turnPIDController.setSetpoint(this.targetAngleDegrees);

		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!turnPIDController.atSetpoint()) {
			double turnOutput = turnPIDController.calculate(driveSubsystem.getHeading());
			driveSubsystem.drive(0, 0, turnOutput);
		} else {
			driveSubsystem.drive(0, 0, 0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// sets motors speeds back to zero when event is complete
		driveSubsystem.drive(0, 0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return turnPIDController.atSetpoint();
	}
}