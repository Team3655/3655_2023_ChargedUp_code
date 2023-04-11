// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class IntakeWheels {

	private Servo rightServo, leftServo;

	public IntakeWheels(int rightPort, int leftPort) {
		rightServo = new Servo(rightPort);
		leftServo = new Servo(leftPort);
	}

	// region setters

	/** sets both intake wheels to be drivin inward for intaking a piece */
	public void Intake() {
		rightServo.set(-1);
		leftServo.set(1);
	}

	/** sets both intake wheels to be drivin outward for releasing a piece */
	public void OutTake() {
		rightServo.set(1);
		leftServo.set(-1);
	}

	/** stops both wheels to reduce power draw and noise */
	public void disable() {
		rightServo.setDisabled();
		leftServo.setDisabled();
	}
	// endregion

	// region getters 
	
	// endregion
}
