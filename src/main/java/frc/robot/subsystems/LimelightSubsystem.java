// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mechanisms.Limelight;

public class LimelightSubsystem extends SubsystemBase {

	public Limelight limelight;

	/** Creates a new LimelightSubsystem */
	public LimelightSubsystem() {
		limelight = new Limelight();
		limelight.setLedMode(1);
	}


	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}


	@Override
	public void periodic() {
	}

}
