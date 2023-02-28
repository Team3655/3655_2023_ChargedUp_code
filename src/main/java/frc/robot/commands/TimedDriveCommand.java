// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedDriveCommand extends CommandBase {

	double ySpeed;
	double xSpeed;
	double rotSpeed;
	double timeMilis;
	DriveSubsystem driveSubsystem;
	Timer timer;

	/** Creates a new TimedDriveCommand. */
	public TimedDriveCommand(double xSpeed, double ySpeed, double rotSpeed, double timeMilis, DriveSubsystem driveSubsystem) {

		this.ySpeed = ySpeed;
		this.xSpeed = xSpeed;
		this.rotSpeed = rotSpeed;
		this.timeMilis = timeMilis;
		this.driveSubsystem = driveSubsystem;

		timer = new Timer();

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		driveSubsystem.drive(xSpeed, ySpeed, rotSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveSubsystem.drive(0, 0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (Units.secondsToMilliseconds(timer.get()) >= timeMilis);
	}
}
