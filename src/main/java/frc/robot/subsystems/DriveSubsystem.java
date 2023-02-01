// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
	}

	private final SwerveModule m_frontLeft = new SwerveModule(
			ModuleConstants.kFrontLeftDriveMotorPort,
			ModuleConstants.kFrontLeftTurningMotorPort,
			ModuleConstants.kFrontLeftTurningEncoderPort,
			ModuleConstants.kFrontLeftAngleZero);

	private final SwerveModule m_rearLeft = new SwerveModule(
			ModuleConstants.kRearLeftDriveMotorPort,
			ModuleConstants.kRearLeftTurningMotorPort,
			ModuleConstants.kRearLeftTurningEncoderPort,
			ModuleConstants.kRearLeftAngleZero);

	private final SwerveModule m_frontRight = new SwerveModule(
			ModuleConstants.kFrontRightDriveMotorPort,
			ModuleConstants.kFrontRightTurningMotorPort,
			ModuleConstants.kFrontRightTurningEncoderPort,
			ModuleConstants.kFrontRightAngleZero);

	private final SwerveModule m_rearRight = new SwerveModule(
			ModuleConstants.kRearRightDriveMotorPort,
			ModuleConstants.kRearRightTurningMotorPort,
			ModuleConstants.kRearRightTurningEncoderPort,
			ModuleConstants.kRearRightAngleZero);

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

		// SmartDashboard.putNumber("FL Turn Output", m_frontLeft.getTurnOutput(swerveModuleStates[0]));
		// SmartDashboard.putNumber("FR Turn Output", m_frontRight.getTurnOutput(swerveModuleStates[1]));
		// SmartDashboard.putNumber("RL Turn Output", m_rearLeft.getTurnOutput(swerveModuleStates[2]));
		// SmartDashboard.putNumber("RR Turn Output", m_rearRight.getTurnOutput(swerveModuleStates[3]));

		// SmartDashboard.putNumber("FL Drive Output", m_frontLeft.getDriveOutput(swerveModuleStates[0]));
		// SmartDashboard.putNumber("FR Drive Output", m_frontRight.getDriveOutput(swerveModuleStates[1]));
		// SmartDashboard.putNumber("RL Drive Output", m_rearLeft.getDriveOutput(swerveModuleStates[2]));
		// SmartDashboard.putNumber("RR Drive Output", m_rearRight.getDriveOutput(swerveModuleStates[3]));
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

	public double getFrontLeftHeading() {
		return m_frontLeft.getEncoderHeading();
	}

	public double getRearLeftHeading() {
		return m_rearLeft.getEncoderHeading();
	}

	public double getFrontRightHeading() {
		return m_frontRight.getEncoderHeading();
	}

	public double getRearRightHeading() {
		return m_rearRight.getEncoderHeading();
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