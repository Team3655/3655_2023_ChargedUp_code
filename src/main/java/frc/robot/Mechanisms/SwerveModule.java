// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.TractorToolbox.SparkMaxMaker;
import frc.lib.TractorToolbox.TractorParts.PIDGains;
import frc.lib.TractorToolbox.TractorParts.SwerveConstants;
import frc.lib.TractorToolbox.TractorParts.SwerveModuleConstants;

public class SwerveModule {

	private final CANSparkMax turnMotor;
	private final CANSparkMax leaderDriveMotor;
	private final CANSparkMax followerDriveMotor;

	// private final CANCoder absoluteEncoder;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnEncoder;
	private final AbsoluteEncoder throughBore;

	private final SparkMaxPIDController drivePID;
	private final SparkMaxPIDController turnPID;

	private final double angleZeroOffset;

	private final String moduleName;

	private SwerveModuleState optimizedState;

	private final SwerveConstants swerveConstants;

	/** Creates a new SwerveModule. */
	public SwerveModule(
			String moduleName,
			SwerveModuleConstants moduleConstants,
			SwerveConstants swerveConstants,
			PIDGains kmoduleturninggains,
			PIDGains kmoduledrivegains) {

		this.swerveConstants = swerveConstants;
		this.moduleName = moduleName;
		this.angleZeroOffset = Units.degreesToRadians(moduleConstants.kAngleZeroOffset);
		optimizedState = new SwerveModuleState();

		// Initialize the motors
		// Initialize turning motor, encoder, and PID
		// Turn Motor
		turnMotor = SparkMaxMaker.createSparkMax(moduleConstants.kTurnMotorID);
		turnMotor.setInverted(true);
		turnMotor.setIdleMode(IdleMode.kCoast);
		turnMotor.setSmartCurrentLimit(swerveConstants.kTurnSmartCurrentLimit);
		// Turn Encoder
		turnEncoder = turnMotor.getEncoder();
		// radians
		turnEncoder.setPositionConversionFactor((2 * Math.PI) * swerveConstants.kTurnGearRatio); 
		// radians per second
		turnEncoder.setVelocityConversionFactor((2 * Math.PI) * swerveConstants.kTurnGearRatio * (1d / 60d));
		// Turn PID
		turnPID = turnMotor.getPIDController();
		turnPID.setP(kmoduleturninggains.kP);
		turnPID.setI(kmoduleturninggains.kI);
		turnPID.setD(kmoduleturninggains.kD);
		turnPID.setPositionPIDWrappingEnabled(true);
		turnPID.setPositionPIDWrappingMinInput(0);
		turnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);

		// Leader Drive Motor
		leaderDriveMotor = SparkMaxMaker.createSparkMax(moduleConstants.kLeaderDriveMotorID);
		leaderDriveMotor.setIdleMode(IdleMode.kBrake);
		leaderDriveMotor.setSmartCurrentLimit(swerveConstants.kDriveSmartCurrentLimit);
		// Leader Drive Encoder
		driveEncoder = leaderDriveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(
				swerveConstants.kDriveGearRatio * swerveConstants.kWheelCircumference); // meters
		driveEncoder.setVelocityConversionFactor(
				swerveConstants.kDriveGearRatio
						* swerveConstants.kWheelCircumference
						* (1d / 60d)); // meters per second

		// Leader Drive PID
		drivePID = leaderDriveMotor.getPIDController();
		drivePID.setP(kmoduledrivegains.kP);
		drivePID.setI(kmoduledrivegains.kI);
		drivePID.setD(kmoduledrivegains.kD);
		drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		drivePID.setSmartMotionMaxAccel(swerveConstants.kMaxModuleAccelMetersPerSecond, 0);
		drivePID.setSmartMotionMaxVelocity(swerveConstants.kMaxModuleSpeedMetersPerSecond, 0);
		drivePID.setFF(swerveConstants.kDriveFeedForward);

		// Follower Drive Motor
		followerDriveMotor = SparkMaxMaker.createSparkMax(moduleConstants.kFolloweDriveMotorID);
		followerDriveMotor.follow(leaderDriveMotor, true);
		followerDriveMotor.setSmartCurrentLimit(swerveConstants.kDriveSmartCurrentLimit);

		// Absolute encoder
		throughBore = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
		throughBore.setPositionConversionFactor((2 * Math.PI));
		throughBore.setVelocityConversionFactor((2 * Math.PI) / 60d);

		leaderDriveMotor.burnFlash();
		followerDriveMotor.burnFlash();
		turnMotor.burnFlash();

		turnPID.setFeedbackDevice(throughBore);
	}

	// region: getters

	/**
	 * returns the current heading of the module from the absolute encoder in
	 * radians
	 */
	public double getAbsoluteHeading() {
		return throughBore.getPosition() - angleZeroOffset;
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	/**
	 * returns the error from the current angle to optimized angle of the module in
	 * radians
	 */
	public double getAngleError() {
		return optimizedState.angle.getRadians() - getAbsoluteHeading();
	}

	public SwerveModulePosition getPosition() {
		// Apply angle offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(
				driveEncoder.getPosition(),
				new Rotation2d(throughBore.getPosition() - angleZeroOffset));
	}
	// endregion: getters

	// region: Setters
	public void setDesiredState(SwerveModuleState desiredState) {
		setDesiredState(desiredState, false);
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isTurbo) {

		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angleZeroOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees to the
		// desired state
		optimizedState = SwerveModuleState.optimize(
				correctedDesiredState,
				new Rotation2d(throughBore.getPosition()));

		if (optimizedState.speedMetersPerSecond <= 0) {
			drivePID.setIAccum(0);
		}

		if (isTurbo) {
			double turboSpeed = Math.copySign(
					swerveConstants.kMaxModuleSpeedMetersPerSecond,
					optimizedState.speedMetersPerSecond);

			drivePID.setReference(turboSpeed, ControlType.kSmartVelocity);
		} else {
			drivePID.setReference(
					optimizedState.speedMetersPerSecond,
					ControlType.kSmartVelocity);
		}

		turnPID.setReference(
				optimizedState.angle.getRadians(),
				ControlType.kPosition);

	}

	public void stopMotors() {
		leaderDriveMotor.stopMotor();
		turnMotor.stopMotor();
	}

	public void setAngleOffset() {
		turnEncoder.setPosition(throughBore.getPosition() - Units.degreesToRadians(angleZeroOffset));
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber(moduleName + " Angle Error", Units.radiansToDegrees(getAngleError()));
		SmartDashboard.putNumber(moduleName + " Offset", angleZeroOffset);
		SmartDashboard.putNumber(moduleName + " Drive Current Draw", leaderDriveMotor.getOutputCurrent());
		SmartDashboard.putNumber(moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(moduleName + " Turn Motor Output", turnMotor.getAppliedOutput());
		SmartDashboard.putNumber(moduleName + " Target Velocity", optimizedState.speedMetersPerSecond);
		SmartDashboard.putNumber(moduleName + " Velocity", driveEncoder.getVelocity());
		SmartDashboard.putNumber(moduleName + " Turn Velocity", turnEncoder.getVelocity());
		SmartDashboard.putNumber(moduleName + " Absolute Angle", Units.radiansToDegrees(getAbsoluteHeading()));
	}

	// endregion: setters

}
