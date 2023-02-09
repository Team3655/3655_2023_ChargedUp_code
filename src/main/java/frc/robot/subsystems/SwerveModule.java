// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */

	private final CANSparkMax m_driveMotor;
	private final CANSparkMax m_turningMotor;

	private final CANCoder m_absoluteEncoder;
	private final RelativeEncoder m_angularEncoder;
	private final RelativeEncoder m_driveEncoder;

	private final SparkMaxPIDController m_angularPID;
	private final SparkMaxPIDController m_drivePID;

	private final double m_angleZero;

	private final String m_moduleName;

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int turningEncoderPorts,
			double angleZero,
			double[] angularPID,
			double[] drivePID) {

		m_moduleName = moduleName;
		
		// Initialize the motors
		m_angleZero = angleZero;

		m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		m_turningMotor.restoreFactoryDefaults();
		m_driveMotor.restoreFactoryDefaults();

		// Initalize CANcoder
		m_absoluteEncoder = new CANCoder(turningEncoderPorts);
		
		m_absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		m_absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		m_absoluteEncoder.configMagnetOffset(0);
		m_absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);

		// Initialize Spark MAX encoders
		m_angularEncoder = m_turningMotor.getEncoder();

		m_turningMotor.setInverted(true);

		m_angularEncoder.setPositionConversionFactor((ModuleConstants.kturnGearRatio) * 2 * Math.PI); // radians
		m_angularEncoder.setVelocityConversionFactor(
				ModuleConstants.kturnGearRatio
						* 2 * Math.PI
						* (1 / 60)); // radians per second

		// m_angularEncoder.setPosition(Math.toRadians(m_absoluteEncoder.getAbsolutePosition())
		// - Math.toRadians(angleZero));

		m_driveEncoder = m_driveMotor.getEncoder();
		m_driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatio * 2 * Math.PI); // meters
		m_driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatio
						* ModuleConstants.kwheelCircumference
						* (1 / 60)); // meters per second

		// Initialize PID's
		m_angularPID = m_turningMotor.getPIDController();
		m_angularPID.setP(angularPID[0]);
		m_angularPID.setI(angularPID[1]);
		m_angularPID.setD(angularPID[2]);

		m_drivePID = m_driveMotor.getPIDController();
		m_drivePID.setP(drivePID[0]);
		m_drivePID.setI(drivePID[1]);
		m_drivePID.setD(drivePID[2]);

		m_angularPID.setFF(DriveConstants.ksTurning);
		m_drivePID.setFF(DriveConstants.kvVoltSecondsPerMeter);

		m_angularPID.setFeedbackDevice(m_turningMotor.getEncoder());
		m_drivePID.setFeedbackDevice(m_driveMotor.getEncoder());

		m_angularPID.setPositionPIDWrappingEnabled(true);
		m_angularPID.setPositionPIDWrappingMinInput(Math.toRadians(0));
		m_angularPID.setPositionPIDWrappingMaxInput(Math.toRadians(360));

		m_angularPID.setOutputRange(-1, 1);
		m_drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors - prevents disabling/brownouts
		m_driveMotor.setIdleMode(IdleMode.kBrake);
		m_turningMotor.setIdleMode(IdleMode.kBrake);
		m_turningMotor.setSmartCurrentLimit(30);
		m_driveMotor.setSmartCurrentLimit(30);

		resetAngleToAbsolute();


		SmartDashboard.putNumber(this.m_moduleName + " Init Spark Enc", resetAngleToAbsolute());
		

	}

	public double resetAngleToAbsolute() {
		double angle = Math.toRadians(m_absoluteEncoder.getAbsolutePosition() - this.m_angleZero);
		SmartDashboard.putString(m_moduleName + " Abs. Error", m_absoluteEncoder.getLastError().toString());
		m_angularEncoder.setPosition(angle);

		return this.m_angularEncoder.getPosition();
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return m_absoluteEncoder.getAbsolutePosition();
	}

	public double getRelativeHeading() {
		return Math.toDegrees(m_angularEncoder.getPosition());
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		double m_moduleAngleRadians = m_angularEncoder.getPosition();

		double m_distanceMeters = m_driveEncoder.getPosition();

		return new SwerveModulePosition(m_distanceMeters, new Rotation2d(m_moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {

		double m_moduleAngleRadians = m_angularEncoder.getPosition();
		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(m_moduleAngleRadians));

		m_angularPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

		m_drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity);

	}

	public void resetEncoders() {
		m_angularEncoder.setPosition(0);
		m_driveMotor.getEncoder().setPosition(0);
	}

	public void putConversionFactors(){
		SmartDashboard.putNumber(m_moduleName + " A P Conversion", m_angularEncoder.getPositionConversionFactor());
		SmartDashboard.putNumber(m_moduleName + " A V Conversion", m_angularEncoder.getVelocityConversionFactor());
		SmartDashboard.putNumber(m_moduleName + " D V Conversion", m_driveEncoder.getVelocityConversionFactor());
	}

	@Override
	public void periodic() {

		// This method will be called once per scheduler run

	}
}
