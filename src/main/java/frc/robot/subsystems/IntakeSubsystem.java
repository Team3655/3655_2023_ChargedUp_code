// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Mechanisms.Vaccum;

public class IntakeSubsystem extends SubsystemBase {

	private Vaccum centerSucker;
	private Vaccum sideSucker;

	private PneumaticHub pneumaticHub;
	private Solenoid centerSolenoid;
	private Solenoid sideSolenoid;

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

		pneumaticHub = new PneumaticHub(IntakeConstants.kPnemnaticHubPort);

		centerSolenoid = pneumaticHub.makeSolenoid(IntakeConstants.kCenterSolenoidPort);
		sideSolenoid = pneumaticHub.makeSolenoid(IntakeConstants.kSideSolenoidPort);

		centerSucker = new Vaccum(
				IntakeConstants.kCenterSuckerPort,
				IntakeConstants.kCenterSuckerCurrentLimit,
				true,
				centerSolenoid);

		sideSucker = new Vaccum(
				IntakeConstants.kSideSuckerPort,
				IntakeConstants.kSideSuckerCurrentLimit,
				true,
				sideSolenoid);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// region commands
	public InstantCommand startSuckingCommand() {
		return new InstantCommand(
				() -> startSucking());
	}

	public void startSucking() {
		centerSucker.suck(IntakeConstants.kCenterSuckerSetpoint);
		// sideSucker.suck(IntakeConstants.kCenterSuckerSetpoint);
	}

	public InstantCommand stopSuckingCommand() {
		return new InstantCommand(
				() -> stopSucking());
	}

	public void stopSucking() {
		centerSucker.drop();
		sideSucker.drop();
	}

}
