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

	private Solenoid dumpValveSolenoid;
	private Solenoid sealerSolenoid;

	public Vaccum(int pumpID, int currentLimit, boolean breakMode, Solenoid dumpValve, Solenoid sealerSolenoid) {

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

		this.dumpValveSolenoid = dumpValve;
		this.sealerSolenoid = sealerSolenoid;
	}


	// region: setters
	public void suck(double suckSpeed) {
		dumpValveSolenoid.set(false);
		sealerSolenoid.set(true);
		pumpMotor.set(suckSpeed);
	}

	public void drop() {
		dumpValveSolenoid.set(true);
		sealerSolenoid.set(false);
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
