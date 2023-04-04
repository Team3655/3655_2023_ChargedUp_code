// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

/** sdfsds */
public class Vaccum {

	private CANSparkMax pumpMotor;
	private RelativeEncoder pumpEncoder;

	private Solenoid dumpValveSolenoid1;
	private Solenoid dumpValveSolenoid2;

	public Vaccum(int pumpID, int currentLimit, boolean breakMode, Solenoid dumpValve1, Solenoid dumpValve2) {

		pumpMotor = new CANSparkMax(pumpID, MotorType.kBrushless);
		pumpMotor.restoreFactoryDefaults();
		pumpMotor.setSmartCurrentLimit(currentLimit);
		pumpMotor.setSecondaryCurrentLimit(currentLimit);

		pumpEncoder = pumpMotor.getEncoder();

		if (breakMode) {
			pumpMotor.setIdleMode(IdleMode.kBrake);
		} else {
			pumpMotor.setIdleMode(IdleMode.kCoast);
		}

		this.dumpValveSolenoid1 = dumpValve1;
		this.dumpValveSolenoid2 = dumpValve2;
	}


	// region: setters
	public void suck(double suckSpeed) {
		dumpValveSolenoid1.set(false);
		dumpValveSolenoid2.set(false);
		pumpMotor.set(suckSpeed);
	}

	public void drop() {
		dumpValveSolenoid1.set(true);
		dumpValveSolenoid2.set(true);
		pumpMotor.stopMotor();
	}
	// endregion

	// region: getters
	public double getRPM() {
		return pumpEncoder.getVelocity();
	}

	public double getMotorCurrentDraw() {
		return pumpMotor.getOutputCurrent();
	}
	// endregion

}
