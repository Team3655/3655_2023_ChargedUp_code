// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DISCLAIMER! THIS DRIVE_SUBSYSTEM HAS MANY BUGS AND SHOULD NOT BE USED AS A REFRANCE!

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Mechanisms.SwerveModule;
import frc.robot.TractorToolbox.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {

	private boolean fieldRelative = true;
	private boolean gyroTurning = false;
	private double targetRotationDegrees;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule rearLeft;
	private final SwerveModule rearRight;

	private SwerveModulePosition[] swervePosition;

	// Initalizing the gyro sensor
	private final WPI_Pigeon2 gyro;

	// Odeometry class for tracking robot pose
	SwerveDriveOdometry odometry;

	// PID controller for gyro turning
	private ProfiledPIDController gyroTurnPidController;

	private Field2d field;

	private SwerveDrivePoseEstimator poseEstimator;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {

		frontLeft = new SwerveModule(
				"FL",
				ModuleConstants.kFrontLeftDriveMotorPort,
				ModuleConstants.kFrontLeftTurningMotorPort,
				ModuleConstants.kFrontLeftTurningEncoderPort,
				ModuleConstants.kFrontLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		frontRight = new SwerveModule(
				"FR",
				ModuleConstants.kFrontRightDriveMotorPort,
				ModuleConstants.kFrontRightTurningMotorPort,
				ModuleConstants.kFrontRightTurningEncoderPort,
				ModuleConstants.kFrontRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		rearLeft = new SwerveModule(
				"RL",
				ModuleConstants.kRearLeftDriveMotorPort,
				ModuleConstants.kRearLeftTurningMotorPort,
				ModuleConstants.kRearLeftTurningEncoderPort,
				ModuleConstants.kRearLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		rearRight = new SwerveModule(
				"RR",
				ModuleConstants.kRearRightDriveMotorPort,
				ModuleConstants.kRearRightTurningMotorPort,
				ModuleConstants.kRearRightTurningEncoderPort,
				ModuleConstants.kRearRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		gyro = new WPI_Pigeon2(DriveConstants.kPigeonPort, Constants.kCanivoreCANBusName);
		gyro.setYaw(0);

		odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePosition);

		field = new Field2d();

		gyroTurnPidController = new ProfiledPIDController(
				DriveConstants.kGyroTurningGains.kP,
				DriveConstants.kGyroTurningGains.kI,
				DriveConstants.kGyroTurningGains.kD,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxTurningVelocityDegrees,
						DriveConstants.kMaxTurningAcceleratonDegrees));

		gyroTurnPidController.enableContinuousInput(-180, 180);
		gyroTurnPidController.setTolerance(DriveConstants.kGyroTurnTolerance);

		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePosition,
				new Pose2d());

		targetRotationDegrees = 0;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		
		updateOdometry();

		if (DriverStation.isDisabled()) {
			// frontLeft.resetEncoders();
			// frontRight.resetEncoders();
			// rearLeft.resetEncoders();
			// rearRight.resetEncoders();
		}

		SmartDashboard.putNumber("FL Absolute", frontLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("FR Absolute", frontRight.getAbsoluteHeading());
		SmartDashboard.putNumber("RL Absolute", rearLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("RR Absolute", rearRight.getAbsoluteHeading());

		SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
		SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
		SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
		SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);

		SmartDashboard.putNumber("Gyro yaw", gyro.getYaw());
		SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
		SmartDashboard.putNumber("Gyro roll", gyro.getRoll());

		SmartDashboard.putNumber("FL Meters", frontLeft.getDistanceMeters());
		SmartDashboard.putNumber("FR Meters", frontRight.getDistanceMeters());
		SmartDashboard.putNumber("RL Meters", rearLeft.getDistanceMeters());
		SmartDashboard.putNumber("RR Meters", rearRight.getDistanceMeters());

		SmartDashboard.putData("field", field);
		SmartDashboard.putNumber("2D Gyro", odometry.getPoseMeters().getRotation().getDegrees());
		SmartDashboard.putNumber("2D X", getPose().getX());
		SmartDashboard.putNumber("2D Y", getPose().getY());
	}

	// region getters
	public double getHeading() {
		return gyro.getRotation2d().getDegrees();
	}

	public double getHeading360() {
		return (gyro.getRotation2d().getDegrees() % 360);
	}

	public double getRoll() {
		return gyro.getRoll();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public Pose2d getPoseEstimatorPose2d() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
				gyro.getRotation2d(),
				swervePosition,
				pose);
	}
	// endregion

	// region setter

	public void lockWheels() {
		double rot = DriveConstants.kMaxRPM;

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				new ChassisSpeeds(0, 0, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, 0);

		setModuleStates(swerveModuleStates);
	}

	public void robotCentricDrive(double xSpeed, double ySpeed, double rot) {
		setFieldCentric(false);
		drive(xSpeed, ySpeed, rot);
		setFieldCentric(true);
	}

	public void drive(double xSpeed, double ySpeed, double rot) {
		drive(xSpeed, ySpeed, rot, false, false);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean isTurbo, boolean isSneak) {

		double maxSpeed;

		if (isSneak) {
			maxSpeed = DriveConstants.kMaxSneakMetersPerSecond;
		} else if (isTurbo) {
			maxSpeed = DriveConstants.kMaxTurboMetersPerSecond;
		} else {
			maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
		}

		// Apply deadbands to inputs
		xSpeed *= maxSpeed;
		ySpeed *= maxSpeed;

		if (gyroTurning) {
			targetRotationDegrees += rot;
			rot = gyroTurnPidController.calculate(getHeading360(), targetRotationDegrees);
		} else {
			rot *= DriveConstants.kMaxRPM;
		}

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));

		setModuleStates(swerveModuleStates);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxModuleSpeedMetersPerSecond);

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);
	}

	public void updateOdometry() {
		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		odometry.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition);

		poseEstimator.update(gyro.getRotation2d(), swervePosition);

		if (LimelightHelpers.getTV("")) {
			Pose2d llPose2d = LimelightHelpers.getBotPose2d_wpiRed("");
			poseEstimator.addVisionMeasurement(
				llPose2d, 
				Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("") - LimelightHelpers.getLatency_Pipeline(""));
		}

		field.setRobotPose(odometry.getPoseMeters());
	}

	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public void setHeading(double heading) {
		gyro.setYaw(heading);
	}

	public CommandBase toggleFieldCentric() {
		return runOnce(() -> {
			fieldRelative = !fieldRelative;
		});
	}

	public void setFieldCentric(boolean fieldCentric) {
		fieldRelative = fieldCentric;
	}

	public void stopMotors() {
		frontLeft.stopMotors();
		frontRight.stopMotors();
		rearLeft.stopMotors();
		rearRight.stopMotors();
	}

	// endregion

}
