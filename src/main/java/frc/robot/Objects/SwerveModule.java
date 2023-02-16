// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Objects;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */

	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private final CANCoder absoluteEncoder;
	private final RelativeEncoder angularEncoder;
	private final RelativeEncoder driveEncoder;

	private final SparkMaxPIDController angularPID;
	private final SparkMaxPIDController drivePID;

	public final double angleZero;

	private final String moduleName;


	private final PIDController m_turningPIDController = new PIDController(
		ModuleConstants.kAngularPID[0],
		ModuleConstants.kAngularPID[1],
		ModuleConstants.kAngularPID[2]);

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
		ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			double[] angularPID,
			double[] drivePID) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		// Initialize the motors
		driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		turningMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		turningMotor.setInverted(true);
		driveMotor.setInverted(true);


		// Initalize CANcoder
		absoluteEncoder = new CANCoder(absoluteEncoderPort);
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);

		// Initialize Spark MAX encoders
		angularEncoder = turningMotor.getEncoder();
		angularEncoder.setPositionConversionFactor(ModuleConstants.kturnGearRatio * 2d * Math.PI); // radians
		angularEncoder.setVelocityConversionFactor(
				ModuleConstants.kturnGearRatio
						* 2 * Math.PI
						* (1 / 60)); // radians per second

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatio * ModuleConstants.kwheelCircumference); // meters
		driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatio
						* ModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		// Initialize PID's
		this.angularPID = turningMotor.getPIDController();
		this.angularPID.setP(angularPID[0]);
		this.angularPID.setI(angularPID[1]);
		this.angularPID.setD(angularPID[2]);

		this.drivePID = driveMotor.getPIDController();
		this.drivePID.setP(drivePID[0]);
		this.drivePID.setI(drivePID[1]);
		this.drivePID.setD(drivePID[2]);

		//this.angularPID.setFF(ModuleConstants.kTurnFeedForward);
		this.drivePID.setFF(ModuleConstants.kDriveFeedForward);

		this.angularPID.setFeedbackDevice(turningMotor.getEncoder());
		this.drivePID.setFeedbackDevice(driveMotor.getEncoder());

		// this.angularPID.setPositionPIDWrappingEnabled(true);
		// this.angularPID.setPositionPIDWrappingMinInput(Math.toRadians(0));
		// this.angularPID.setPositionPIDWrappingMaxInput(Math.toRadians(360));

		// this.angularPID.setOutputRange(-1, 1);
		this.drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors
		driveMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setSmartCurrentLimit(30);
		driveMotor.setSmartCurrentLimit(30);

		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		SmartDashboard.putNumber(this.moduleName + " Offset", angleZero);
		SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition();
	}

	public double getRelativeHeading() {
		return Math.toDegrees(angularEncoder.getPosition());
	}

	public double getDistanceMeters(){
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		//double m_moduleAngleRadians = angularEncoder.getPosition();
		double m_moduleAngleRadians = absoluteEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

		double m_distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(m_distanceMeters, new Rotation2d(m_moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {

		//double m_moduleAngleRadians = angularEncoder.getPosition();
		double m_moduleAngleRadians = absoluteEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(m_moduleAngleRadians));



		final var angularFFOutput = turnFeedForward.calculate(m_turningPIDController.getSetpoint());
		final var angularPIDOutput = m_turningPIDController.calculate(m_moduleAngleRadians, optimizedState.angle.getRadians());

		final var turnOutput = angularFFOutput + angularPIDOutput;

		turningMotor.setVoltage(turnOutput);

		drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity);

		SmartDashboard.putNumber(this.moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(this.moduleName + " FF", angularFFOutput);
		SmartDashboard.putNumber(this.moduleName + " PID", angularPIDOutput);
	}

	public void resetEncoders() {
		driveMotor.getEncoder().setPosition(0);
	}

	public void putConversionFactors() {
		SmartDashboard.putNumber(moduleName + " D V Conversion", driveEncoder.getVelocityConversionFactor());
		SmartDashboard.putNumber(moduleName + " D Ang Conv", driveEncoder.getPositionConversionFactor());
	}

	@Override
	public void periodic() {

		// This method will be called once per scheduler run

	}
}
