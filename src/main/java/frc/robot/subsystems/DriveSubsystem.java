// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Objects.SwerveModule;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveSubsystem extends SubsystemBase {

	private boolean fieldRelative=true;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {

	}

	private final SwerveModule frontLeft = new SwerveModule(
			"FL",
			ModuleConstants.kFrontLeftDriveMotorPort,
			ModuleConstants.kFrontLeftTurningMotorPort,
			ModuleConstants.kFrontLeftTurningEncoderPort,
			ModuleConstants.kFrontLeftAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private final SwerveModule frontRight = new SwerveModule(
			"FR",
			ModuleConstants.kFrontRightDriveMotorPort,
			ModuleConstants.kFrontRightTurningMotorPort,
			ModuleConstants.kFrontRightTurningEncoderPort,
			ModuleConstants.kFrontRightAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private final SwerveModule rearLeft = new SwerveModule(
			"RL",
			ModuleConstants.kRearLeftDriveMotorPort,
			ModuleConstants.kRearLeftTurningMotorPort,
			ModuleConstants.kRearLeftTurningEncoderPort,
			ModuleConstants.kRearLeftAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private final SwerveModule rearRight = new SwerveModule(
			"RR",
			ModuleConstants.kRearRightDriveMotorPort,
			ModuleConstants.kRearRightTurningMotorPort,
			ModuleConstants.kRearRightTurningEncoderPort,
			ModuleConstants.kRearRightAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private SwerveModulePosition[] swervePosition = new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearLeft.getPosition(),
			rearRight.getPosition()
	};

	// Initalizing the gyro sensor
	private final WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.kPigeonPort, Constants.kCanivoreCANBusName);

	// Odeometry class for tracking robot pose
	SwerveDriveOdometry odometry = new SwerveDriveOdometry(
			DriveConstants.kDriveKinematics,
			gyro.getRotation2d(),
			swervePosition);

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		odometry.update(
				gyro.getRotation2d(),
				swervePosition);

		frontLeft.putConversionFactors();
		frontRight.putConversionFactors();
		rearRight.putConversionFactors();
		rearLeft.putConversionFactors();

		SmartDashboard.putNumber("FL Absolute", frontLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("FR Absolute", frontRight.getAbsoluteHeading());
		SmartDashboard.putNumber("RL Absolute", rearLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("RR Absolute", rearRight.getAbsoluteHeading());

		SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
		SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
		SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
		SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);

		SmartDashboard.putNumber("FL Relative", frontLeft.getRelativeHeading());
		SmartDashboard.putNumber("FR Relative", frontRight.getRelativeHeading());
		SmartDashboard.putNumber("RL Relative", rearLeft.getRelativeHeading());
		SmartDashboard.putNumber("RR Relative", rearRight.getRelativeHeading());

		SmartDashboard.putNumber("Gyro", gyro.getYaw());

		SmartDashboard.putNumber("FL Meters", frontLeft.getDistanceMeters());
		SmartDashboard.putNumber("FR Meters", frontRight.getDistanceMeters());
		SmartDashboard.putNumber("RL Meters", rearLeft.getDistanceMeters());
		SmartDashboard.putNumber("RR Meters", rearRight.getDistanceMeters());

		SmartDashboard.putNumber("2D Gyro", getHeading());
		SmartDashboard.putNumber("2D X", getPose().getX());
		SmartDashboard.putNumber("2D Y", getPose().getY());
		

	}

	// region getters
	public double getHeading() {
		return gyro.getRotation2d().getDegrees();
	}

	public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
				gyro.getRotation2d(),
				swervePosition,
				pose);
	}
	// endregion

	// region setter
	public void drive(double xSpeed, double ySpeed, double rot) {
		drive(xSpeed, ySpeed, rot, true, false);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean isTurbo, Boolean isSneak) {

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
		
		rot *= DriveConstants.kMaxRPM;

		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates,
			ModuleConstants.kMaxModuleSpeedMetersPerSecond);

		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[3]);

	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);

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

	// public CommandBase toggleFieldCentric() {
	// 	return runOnce(() -> {
	// 		fieldRelative = !fieldRelative;
	// 	});
	// }

	// endregion

	/**************************************************************************/

	public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
		return new SequentialCommandGroup(
				new InstantCommand(() -> {
					// Reset odometry for the first path you run during auto
					if (isFirstPath) {
						this.resetOdometry(trajectory.getInitialHolonomicPose());
					}
				}),
				new PPSwerveControllerCommand(
						trajectory,
						this::getPose, // Pose supplier
						DriveConstants.kDriveKinematics, // SwerveDriveKinematics
						new PIDController(
								AutoConstants.PathPlannerP,
								AutoConstants.PathPlannerI,
								AutoConstants.PathPlannerD), 
						new PIDController(
								AutoConstants.PathPlannerP,
								AutoConstants.PathPlannerI,
								AutoConstants.PathPlannerD), 
						new PIDController(
								AutoConstants.PathPlannerTurnP,
								AutoConstants.PathPlannerTurnI,
								AutoConstants.PathPlannerTurnD), 
						this::setModuleStates, // Module states consumer
						false, 
						this // Requires this drive subsystem
				));
	}

}