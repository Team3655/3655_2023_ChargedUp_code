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
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

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

public class SwerveModule extends SubsystemBase {
	/** Creates a new SwerveModule. */

	private final CANSparkMax m_driveMotor;
	private final CANSparkMax m_turningMotor;

	private final CANCoder m_turnEncoder;
	private final RelativeEncoder m_angularEncoder;

	private final PIDController m_drivePIDController = new PIDController(
			ModuleConstants.kModuleDriveControllerP,
			ModuleConstants.kModuleDriveControllerI,
			ModuleConstants.kModuleDriveControllerD);

	private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
			ModuleConstants.kModuleTurningControllerP,
			ModuleConstants.kModuleTurningControllerI,
			ModuleConstants.kModuleTurningControllerD,
			new TrapezoidProfile.Constraints(
					DriveConstants.kMaxModuleAngularSpeedRadiansPerSecond,
					DriveConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

	SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
			DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			DriveConstants.ksTurning, DriveConstants.kvTurning);

	public SwerveModule(
			int driveMotorChannel,
			int turningMotorChannel,
			int turningEncoderPorts,
			double angleZero,
			double[] angularPID,
			double[] drivePID) {

		// Initialize the motors
		m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		m_turningMotor.restoreFactoryDefaults();
		m_driveMotor.restoreFactoryDefaults();

		m_turningMotor.getPIDController().setP(angularPID[0]);
		m_turningMotor.getPIDController().setI(angularPID[1]);
		m_turningMotor.getPIDController().setD(angularPID[2]);

		m_driveMotor.getPIDController().setP(drivePID[0]);
		m_driveMotor.getPIDController().setI(drivePID[1]);
		m_driveMotor.getPIDController().setD(drivePID[2]);

		m_turningMotor.getPIDController().setFF(DriveConstants.ksTurning);
		m_driveMotor.getPIDController().setFF(DriveConstants.kvVoltSecondsPerMeter);

		m_turningMotor.getPIDController().setFeedbackDevice(m_turningMotor.getEncoder());
		m_driveMotor.getPIDController().setFeedbackDevice(m_driveMotor.getEncoder());

		m_turningMotor.getPIDController().setPositionPIDWrappingEnabled(true);
		m_turningMotor.getPIDController().setPositionPIDWrappingMinInput(Math.toRadians(-180));
		m_turningMotor.getPIDController().setPositionPIDWrappingMaxInput(Math.toRadians(180));

		// Configure current limits for motors - prevents disabling/brownouts
		m_driveMotor.setSecondaryCurrentLimit(30);
		m_driveMotor.setSmartCurrentLimit(30);
		m_driveMotor.setIdleMode(IdleMode.kBrake);
		m_turningMotor.setIdleMode(IdleMode.kBrake);
		m_turningMotor.setSmartCurrentLimit(30);

		// Configure the encoders for both motors
		// CANcoder defaults range 0 to 360. WPILib swerve module has angles from -180
		// to 180
		// Changed range to accomodate this issue

		m_driveMotor.getEncoder().setVelocityConversionFactor(
			ModuleConstants.kdriveGearRatio
			* ModuleConstants.kwheelCircumference
			* (1 / 60));

		m_turnEncoder = new CANCoder(turningEncoderPorts);
		m_turnEncoder.configFactoryDefault();
		m_turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		m_turnEncoder.configMagnetOffset(-1 * angleZero);
		m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);

		// Attempting to use SPARK MAX Encoders instead

		m_angularEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
		//Position normally measured in rotations; convert to radians
		m_angularEncoder.setPositionConversionFactor((1/ModuleConstants.kturnGearRatio) * 2 * Math.PI);
		
		
		m_angularEncoder.setPosition(Math.toRadians(m_turnEncoder.getAbsolutePosition()));
		

		// Set turning PID output to allow the swerve modules to treat the min/max as
		// continuous
		// Optimizes how the modules chooses to go to a desired position
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	// Returns headings of the module
	public double getModuleHeading() {
		double m_turning = m_angularEncoder.getPosition();
		return m_turning;
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		double m_moduleAngleRadians = m_angularEncoder.getPosition();

		double m_distanceMeters = m_driveMotor.getEncoder().getPosition();

		return new SwerveModulePosition(m_distanceMeters, new Rotation2d(m_moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {

		double m_speedMetersPerSecond = m_driveMotor.getEncoder().getVelocity(); 

		double m_moduleAngleRadians = m_angularEncoder.getPosition();
		// Optimize the reference state to avoid spinning further than 90 degreesto
		// desired state
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_moduleAngleRadians));

		m_turningMotor.getPIDController().setReference(
				state.angle.getRadians(),
				ControlType.kPosition);

		m_driveMotor.getPIDController().setReference(
			state.speedMetersPerSecond,
			ControlType.kVelocity);


	}

	public void resetEncoders() {
		//m_turnEncoder.setPosition(0);
		m_angularEncoder.setPosition(0);
		m_driveMotor.getEncoder().setPosition(0);
	}

	public double getEncoderHeading() {
		return Math.toDegrees(m_angularEncoder.getPosition());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
