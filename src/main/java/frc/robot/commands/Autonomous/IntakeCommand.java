// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

	private IntakeSubsystem intakeSubsystem;
	private Timer timer;

	private double timeMilis;
	private boolean suck;

	/** Creates a new SuckCommand. */
	public IntakeCommand(boolean suck, double timeMilis) {

		intakeSubsystem = RobotContainer.intakeSubsystem;

		timer = new Timer();
		this.timeMilis = timeMilis;
		this.suck = suck;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intakeSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (suck) {
			intakeSubsystem.setIntakeState(kIntakeStates.INTAKE);
		} else {
			intakeSubsystem.setIntakeState(kIntakeStates.OUTTAKE);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.startSucking();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (Units.secondsToMilliseconds(timer.get()) >= timeMilis);
	}
}
