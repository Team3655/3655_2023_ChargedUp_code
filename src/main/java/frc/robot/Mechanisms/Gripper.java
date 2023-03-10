// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class Gripper {

	private Servo rightServo, leftServo;

	public Gripper(int rightPort, int leftPort) {
		rightServo = new Servo(rightPort);
		leftServo = new Servo(leftPort);
		closeGriper();
	}

	public void openGriper() {
		rightServo.set(1);
		leftServo.set(0);
	}

	public void closeGriper() {
		rightServo.set(0);
		leftServo.set(1);
	}

}
