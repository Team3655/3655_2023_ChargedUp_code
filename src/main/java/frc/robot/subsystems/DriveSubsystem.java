// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
	}

	private final SwerveModule m_frontLeft = new SwerveModule(
			"FL",
			ModuleConstants.kFrontLeftDriveMotorPort,
			ModuleConstants.kFrontLeftTurningMotorPort,
			ModuleConstants.kFrontLeftTurningEncoderPort,
			ModuleConstants.kFrontLeftAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private final SwerveModule m_frontRight = new SwerveModule(
			"FR",
			ModuleConstants.kFrontRightDriveMotorPort,
			ModuleConstants.kFrontRightTurningMotorPort,
			ModuleConstants.kFrontRightTurningEncoderPort,
			ModuleConstants.kFrontRightAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private final SwerveModule m_rearLeft = new SwerveModule(
			"RL",
			ModuleConstants.kRearLeftDriveMotorPort,
			ModuleConstants.kRearLeftTurningMotorPort,
			ModuleConstants.kRearLeftTurningEncoderPort,
			ModuleConstants.kRearLeftAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	private final SwerveModule m_rearRight = new SwerveModule(
			"RR",
			ModuleConstants.kRearRightDriveMotorPort,
			ModuleConstants.kRearRightTurningMotorPort,
			ModuleConstants.kRearRightTurningEncoderPort,
			ModuleConstants.kRearRightAngleZero,
			ModuleConstants.kAngularPID,
			ModuleConstants.kDrivePID);

	// TODO: Use variable here instead of entries below?
	private SwerveModulePosition[] m_swervePosition = new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_rearLeft.getPosition(),
			m_rearRight.getPosition()
	};

	// Initalizing the gyro sensor
	private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(DriveConstants.kPigeonPort);

	// Odeometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
			DriveConstants.kDriveKinematics,
			m_gyro.getRotation2d(),
			m_swervePosition);

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_odometry.update(
				m_gyro.getRotation2d(),
				m_swervePosition);

		m_frontLeft.putConversionFactors();
		m_frontRight.putConversionFactors();
		m_rearRight.putConversionFactors();
		m_rearLeft.putConversionFactors();

		SmartDashboard.putNumber("Gyro", m_gyro.getAngle());

	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(
				m_gyro.getRotation2d(),
				m_swervePosition,
				pose);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);

		SmartDashboard.putNumber("FL Absolute", m_frontLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("FR Absolute", m_frontRight.getAbsoluteHeading());
		SmartDashboard.putNumber("RL Absolute", m_rearLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("RR Absolute", m_rearRight.getAbsoluteHeading());

		SmartDashboard.putNumber("FL Relative", m_frontLeft.getRelativeHeading());
		SmartDashboard.putNumber("FR Relative", m_frontRight.getRelativeHeading());
		SmartDashboard.putNumber("RL Relative", m_rearLeft.getRelativeHeading());
		SmartDashboard.putNumber("RR Relative", m_rearRight.getRelativeHeading());

	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);

	}

	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	public void zeroHeading() {
		m_gyro.reset();
	}

	public double getHeading() {
		return m_gyro.getRotation2d().getDegrees();
	}

	public double getTurnRate() {
		return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public void resetRelativeEncoders() {
		m_frontLeft.resetAngleToAbsolute();
		m_frontRight.resetAngleToAbsolute();
		m_rearRight.resetAngleToAbsolute();
		m_rearLeft.resetAngleToAbsolute();
	}

	/************************************************************************* */

	public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
		return new SequentialCommandGroup(
				new InstantCommand(() -> {
					// Reset odometry for the first path you run during auto
					if (isFirstPath) {
						this.resetOdometry(traj.getInitialHolonomicPose());
					}
				}),
				new PPSwerveControllerCommand(
						traj,
						this::getPose, // Pose supplier
						DriveConstants.kDriveKinematics, // SwerveDriveKinematics
						new PIDController(
								DashboardSubsystem.PIDConstants.getPlanner_X_kP(),
								0,
								DashboardSubsystem.PIDConstants.getPlanner_X_kD()), // TODO: X controller.
						// Tune these
						// values for your robot. Leaving
						// them 0 will only use
						// feedforwards.
						new PIDController(
								DashboardSubsystem.PIDConstants.getPlanner_Y_kP(),
								0,
								DashboardSubsystem.PIDConstants.getPlanner_Y_kD()), // TODO: controller
						// (usually the
						// same values as X controller)
						new PIDController(
								DashboardSubsystem.PIDConstants.getPlanner_Rot_kP(),
								0,
								DashboardSubsystem.PIDConstants.getPlanner_Rot_kD()), // TODO: Rotation controller.
						// Tune
						// these values for your robot.
						// Leaving them 0 will only use
						// feedforwards.
						this::setModuleStates, // Module states consumer
						true, // Sho uld the path be automatically mirrored depending on alliance
						// color.
						// Optional, defaults to true
						this // Requires this drive subsystem
				));
	}

}