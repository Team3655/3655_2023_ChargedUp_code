// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	
	/** Motors for the Suckers */
	private CANSparkMax mainSucker;

	/** Encoders for Sucker motors */
	private RelativeEncoder mainEncoder;

	/** Color sensor for detectng which type of piece is held */
	private ColorSensorV3 colorSense;

	/** power distribution hub, used for switching the solenoid */
	private PowerDistribution pdh;

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

		pdh = new PowerDistribution(1, ModuleType.kRev);

		// Give Sucker motors their id's
		mainSucker = new CANSparkMax(IntakeConstants.kMainSuckerPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		mainSucker.restoreFactoryDefaults();

		// set current limits
		mainSucker.setSmartCurrentLimit(IntakeConstants.kMainSuckerCurrentLimit);
		mainSucker.setSecondaryCurrentLimit(IntakeConstants.kMainSuckerStallCurrentLimit);

		// sets motor default idle mode
		mainSucker.setIdleMode(IdleMode.kBrake);
		
		mainSucker.set(IntakeConstants.kMainSuckerSetpoint);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// region commands
	public CommandBase stopSucking() {
		return runOnce(
				() -> {
					mainSucker.set(0);
					enableSideSucker();
				});
	}

	public void stopstopSucking() {
		mainSucker.set(0);
	}

	public CommandBase startSucking() {
		return runOnce(
				() -> {
					mainSucker.set(IntakeConstants.kMainSuckerSetpoint);
					disableSideSucker();
				});
	}

	public CommandBase toggleSideSucker() {
		return runOnce(
				() -> {
					if (pdh.getSwitchableChannel()) {
						disableSideSucker();
					} else {
						enableSideSucker();
					}
				});
	}

	public void disableSideSucker() {
		pdh.setSwitchableChannel(false);
	}

	public void enableSideSucker() {
		pdh.setSwitchableChannel(true);
	}

	// endregion

	// region getters
	/**
	 * Checks if the robot is holding a game piece by using the output of the motor
	 *
	 * @return True if the robot is holding a piece
	 */
	public boolean getHasPiece() {

		if (mainEncoder.getVelocity() < IntakeConstants.kHasPieceThreshold) {
			return true;
		}

		return false;
	}

	/**
	 * Checks if the robot is holding a Cone by using the color sensor and
	 * getHasPiece()
	 * 
	 * @return True if the robot is holding a cone
	 */
	public boolean getHasCone() {
		if (getHasPiece() && colorSense.getColor() == Color.kYellow) {
			return true;
		}
		return false;
	}

	/**
	 * Checks if the robot is holding a Cube by using the color sensor and
	 * getHasPiece()
	 * 
	 * @return True if the robot is holding a Cube
	 */
	public boolean getHasCube() {
		if (getHasPiece() && colorSense.getColor() == Color.kPurple) {
			return true;
		}
		return false;
	}

	// endregion
}
