// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

	/** Motors for the Suckers */
	private CANSparkMax m_mainSucker;

	/** PID controlers for Suckers */
	private SparkMaxPIDController m_mainPIDController;

	/** Encoders for Sucker motors */
	private RelativeEncoder m_mainEncoder;

	/** Color sensor for detectng which type of piece is held */
	private ColorSensorV3 m_colorSense;

	/** Used to tell if the suckers are running */
	private boolean isSucking;

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

		// Give Sucker motors their id's
		m_mainSucker = new CANSparkMax(IntakeConstants.kMainSuckerPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		m_mainSucker.restoreFactoryDefaults();

		// set current limits
		m_mainSucker.setSmartCurrentLimit(IntakeConstants.kMainSuckerCurrentTarget);

		// sets motor defaults to break
		m_mainSucker.setIdleMode(IdleMode.kCoast);

		m_mainSucker.stopMotor();

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		m_mainPIDController = m_mainSucker.getPIDController();
		m_mainPIDController.setP(IntakeConstants.kMainSuckerP);
		m_mainPIDController.setOutputRange(0, IntakeConstants.kMainSuckerMaxOutput);

		// Encoder object created to display velocity values
		m_mainEncoder = m_mainSucker.getEncoder();

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// region commands

	public CommandBase toggleIntake() {
		// toggle the state of the sucker
		isSucking = !isSucking;

		// Stops motor if isSucking is false
		return runOnce(
				() -> {

					if (isSucking) {
						m_mainPIDController.setReference(
								IntakeConstants.kMainSuckerCurrentTarget,
								ControlType.kCurrent);
					} else {
						m_mainSucker.stopMotor();
					}

				});
	}

	// endregion

	// region getters

	/**
	 * Checks if the robot is holding a game piece by using the current draw of the
	 * center sucker
	 *
	 * @return True if the robot is holding a piece
	 */
	public boolean getHasPiece() {

		if (m_mainEncoder.getVelocity() < IntakeConstants.kHasPieceRPMThreshold) {
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
		if (getHasPiece() && m_colorSense.getColor() == Color.kYellow) {
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
		if (getHasPiece() && m_colorSense.getColor() == Color.kPurple) {
			return true;
		}
		return false;
	}

	// endregion
}
