// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

	/** Creates the motors for the Suckers */
	private CANSparkMax m_rightSuckerMotor, m_centerSuckerMotor, m_leftSuckerMotor;

	private ColorSensorV3 m_colorSense;

	/** Creates pid controlers for Suckers */
	private SparkMaxPIDController m_centerPIDController, m_sidePIDController;

	/** Creates encoders for Sucker motors */
	private RelativeEncoder m_rightSuckerEncoder, m_centerSuckerEncoder, m_leftSuckerEncoder;

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

		// Give Sucker motors their id's
		m_rightSuckerMotor = new CANSparkMax(IntakeConstants.kRightSuckerPort, MotorType.kBrushless);
		m_centerSuckerMotor = new CANSparkMax(IntakeConstants.kCenterSuckerPort, MotorType.kBrushless);
		m_leftSuckerMotor = new CANSparkMax(IntakeConstants.kLeftSuckerPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		m_rightSuckerMotor.restoreFactoryDefaults();
		m_centerSuckerMotor.restoreFactoryDefaults();
		m_leftSuckerMotor.restoreFactoryDefaults();

		// set current limits TODO: use stallimit with an rpm
		// m_rightSuckerMotor.setSmartCurrentLimit(IntakeConstants.kSideSuckerCurrentLimit);
		// m_centerSuckerMotor.setSmartCurrentLimit(IntakeConstants.kCenterSuckerCurrentLimit);
		// m_leftSuckerMotor.setSmartCurrentLimit(IntakeConstants.kSideSuckerCurrentLimit);

		// sets motor defaults to break
		m_rightSuckerMotor.setIdleMode(IdleMode.kCoast);
		m_centerSuckerMotor.setIdleMode(IdleMode.kCoast);
		m_leftSuckerMotor.setIdleMode(IdleMode.kCoast);

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		m_centerPIDController = m_centerSuckerMotor.getPIDController();
		m_sidePIDController = m_rightSuckerMotor.getPIDController();

		// Encoder object created to display velocity values
		m_rightSuckerEncoder = m_rightSuckerMotor.getEncoder();
		m_centerSuckerEncoder = m_centerSuckerMotor.getEncoder();
		m_leftSuckerEncoder = m_leftSuckerMotor.getEncoder();

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// region getters

	public boolean getHasPiece() {
		if (m_centerSuckerMotor.getOutputCurrent() > IntakeConstants.kHasPieceCurrentThreshold) {
			return true;
		}
		return false;
	}

	public boolean getHasCone() {
		if (getHasPiece() && m_colorSense.getColor() == Color.kYellow) {
			return true;
		}
		return false;
	}

	// endregion
}
