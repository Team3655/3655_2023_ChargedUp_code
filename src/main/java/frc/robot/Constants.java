// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.TractorToolbox.TractorParts.PIDGains;
import frc.lib.TractorToolbox.TractorParts.SwerveConstants;
import frc.lib.TractorToolbox.TractorParts.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.commands.Limelight.LLAlignCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class ModuleConstants {

		public static final int kMaxRezeroAttempts = 10;

		// gains set for R1 SDS mk4i using dual neo motors
		public static final PIDGains kModuleDriveGains = new PIDGains(.075, 0, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(.5, 0, 0.0);

		public static final class GenericModuleConstants {
			// Current limits for the wheels
			public static final int kTurnMotorCurrentLimit = 25;
			public static final int kDriveMotorCurrentLimit = 35;

			// Constants set for the _SDS MK4i_
			public static final double kTurnGearRatio = 1d / (150d / 7d);
			public static final double kDriveGearRatio = 1d / (38250 / 6885);
			public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;

			// The max speeds the modules are capable of
			public static final double kMaxModuleAccelMetersPerSecond = 5;
			public static final double kMaxModuleSpeedMetersPerSecond = 5.35;

			// Retune feedforward values for turning
			// public static final double kvTurning = .43205;
			// public static final double ksTurning = .17161; // Tuned February 2, 2023

			public static final double kDriveFeedForward = .2;

			public static final SwerveConstants kSwerveConstants = new SwerveConstants(
					kTurnMotorCurrentLimit,
					kDriveMotorCurrentLimit,
					kTurnGearRatio,
					kDriveGearRatio,
					kWheelCircumference,
					kMaxModuleAccelMetersPerSecond,
					kMaxModuleSpeedMetersPerSecond,
					kDriveFeedForward);
		}

		// module specific constants
		public static final class FrontLeftModule {
			public static final int kTurningMotorID = 10;
			public static final int kLeaderDriveMotorID = 11;
			public static final int kFollowerDriveMotorID = 12;
			// public static final int kAbsoluteEncoderID = 9;
			public static final double kAngleOffset = 26.325;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}

		public static final class FrontRightModule {
			public static final int kTurningMotorID = 13;
			public static final int kLeaderDriveMotorID = 14;
			public static final int kFollowerDriveMotorID = 15;
			// public static final int kAbsoluteEncoderID = 10;
			public static final double kAngleOffset = 311.163;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}

		public static final class BackLeftModule {
			public static final int kTurningMotorID = 16;
			public static final int kLeaderDriveMotorID = 17;
			public static final int kFollowerDriveMotorID = 18;
			// public static final int kAbsoluteEncoderID = 11;
			public static final double kAngleOffset = 31.957;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}

		public static final class BackRightModule {
			public static final int kTurningMotorID = 19;
			public static final int kLeaderDriveMotorID = 20;
			public static final int kFollowerDriveMotorID = 21;
			// public static final int kAbsoluteEncoderID = 12;
			public static final double kAngleOffset = 44.571;
			public static final SwerveModuleConstants kModuleConstants = new SwerveModuleConstants(
					// kAbsoluteEncoderID,
					kTurningMotorID,
					kLeaderDriveMotorID,
					kFollowerDriveMotorID,
					kAngleOffset);
		}
	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.0;
		public static final double kMaxSpeedMetersPerSecond = 4;

		// this sets turning speed (keep this low) KsKs
		public static final double kMaxRPM = 8;

		public static final int kPigeonPort = 2;

		public static final double kBumperToBumperWidth = Units.inchesToMeters(31);

		public static final double kTrackWidth = Units.inchesToMeters(20); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(20); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

		public static final boolean kGyroReversed = true;
		public static final boolean kFeildCentric = true;

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			public static final PIDGains kPPDriveGains = new PIDGains(8.5, 0, 0);
			public static final PIDGains kPPTurnGains = new PIDGains(3.5, 0, 0);

			public static final double kPPMaxVelocity = 4.00;
			public static final double kPPMaxAcceleration = 2.5;

			public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
				{
					put("TargetTape", new LLAlignCommand(false));
					put("TargetTag", new LLAlignCommand(true));
				}
			};
		}

		public static final double kScoreSequenceDropTime = 3; // in seconds

		public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0.0003, 0);
		public static final double kTurnCommandMaxVelocity = 1;
		public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 0.5;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = 10;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.003, 0, 0);
		public static final double kMaxBalancingVelocity = 1000;
		public static final double kMaxBalancingAcceleration = 5000;

	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

		public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	/**
	 * The constants pertaining to Arm (and sub arms)
	 */
	public static class ArmConstants {

		// NEO turning motor CAN ID's
		public static final int kRightMajorArmPort = 30;
		public static final int kLeftMajorArmPort = 31;
		public static final int kRightMinorArmPort = 32;
		public static final int kLeftMinorArmPort = 33;

		public static final double kMajorArmGearBoxRatio = 100;
		public static final double kMinorArmGearBoxRatio = 100;

		public static final double kMajorArmBeltRatio = 2d / 1d;
		public static final double kMinorArmBeltRatio = 115d / 70d;

		/**
		 * the total number of motor rotations for one 360 degree rotation of the arm
		 */
		public static final double kMajorArmTicks = kMajorArmGearBoxRatio * kMajorArmBeltRatio;
		public static final double kMinorArmTicks = kMinorArmGearBoxRatio * kMinorArmBeltRatio;

		/**
		 * The radius of each arms rotation in inches (from center of rotation to next
		 * arms center of rotation)
		 */
		public static final int kMajorArmLength = 38;
		public static final int kMinorArmLength = 23;

		// current limits of the arms
		public static final int kMajorArmCurrentLimit = 5;
		public static final int kMinorArmCurrentLimit = 8;

		// speed limits for the arms
		public static final double kMajorPIDOutputLimit = 1;
		public static final double kMinorPIDOutputLimit = 1;

		public static final double kMaxMajorVelRadiansPerSec = (Math.PI * 10) * 60;
		public static final double kMaxMajorAccelRadiansPerSec = (Math.PI * 6.25) * 60;

		public static final double kMaxMinorVelRadiansPerSec = (Math.PI * 10) * 60;
		public static final double kMaxMinorAccelRadiansPerSec = (Math.PI * 8) * 60;

		// angle limits for the arms
		public static final double kMajorArmConstraints = 110;
		public static final double kMinorArmConstraints = 180;

		// Arm PID constants
		public static final PIDGains kMajorArmGains = new PIDGains(0.0035, 0.0000025, 0.002);

		public static final PIDGains kMinorArmGains = new PIDGains(0.002, 0.0000008, 0.001);

		public static enum kArmPoses {
			TUCKED,
			LOW_SCORE,
			MID_SCORE,
			HIGH_SCORE,
			LOW_INTAKE,
			MID_INTAKE,
			HIGH_INTAKE,
			DRIVER_CONTROL,
			KICK_FRONT,
			KICK_BACK,
		}

		public static final HashMap<kArmPoses, double[]> kArmStatesMap = new HashMap<kArmPoses, double[]>() {
			{
				put(kArmPoses.TUCKED, new double[] { 0, 0 });
				put(kArmPoses.LOW_SCORE, new double[] { 0, 80 });
				put(kArmPoses.MID_SCORE, new double[] { 70, 60 });
				put(kArmPoses.HIGH_SCORE, new double[] { 100, 75 });
				put(kArmPoses.LOW_INTAKE, new double[] { 10, -98 });
				put(kArmPoses.MID_INTAKE, new double[] { 13, 33 });
				put(kArmPoses.HIGH_INTAKE, new double[] { 100, 100 });
				put(kArmPoses.DRIVER_CONTROL, new double[] { 0, 0 });
				put(kArmPoses.KICK_FRONT, new double[] { 35, 0 });
				put(kArmPoses.KICK_BACK, new double[] { -35, 0 });
			}

		};

	}

	public static class IntakeConstants {

		public static final int kRightIntakeWheelPort = 8;
		public static final int kLeftIntakeWheelPort = 9;

		public static final int kPnemnaticHubPort = 50;

		public static final int kCenterDumpSolenoidPort = 0;
		public static final int kCenterSealerSolenoidPort = 1;

		// NEO Sucker motor CAN ID's
		public static final int kCenterSuckerID = 1;

		public static final int kCenterSuckerCurrentLimit = 6;

		public static final double kCenterSuckerSetpoint = 0.65;
		public static final int kHasConeThreshold = 3530;

		public static enum kIntakeStates {
			IDLE,
			INTAKE,
			OUTTAKE,
			DISABLED
		}

		public static final HashMap<kArmPoses, kIntakeStates> kArmStateToIntakeStateMap = new HashMap<kArmPoses, kIntakeStates>() {
			{
				put(kArmPoses.TUCKED, kIntakeStates.IDLE);
				put(kArmPoses.LOW_SCORE, kIntakeStates.IDLE);
				put(kArmPoses.MID_SCORE, kIntakeStates.IDLE);
				put(kArmPoses.HIGH_SCORE, kIntakeStates.IDLE);
				put(kArmPoses.LOW_INTAKE, kIntakeStates.INTAKE);
				put(kArmPoses.MID_INTAKE, kIntakeStates.INTAKE);
				put(kArmPoses.HIGH_INTAKE, kIntakeStates.INTAKE);
				put(kArmPoses.DRIVER_CONTROL, kIntakeStates.INTAKE);
				put(kArmPoses.KICK_FRONT, kIntakeStates.IDLE);
				put(kArmPoses.KICK_BACK, kIntakeStates.IDLE);
			}
		};

	}

	public static class LimelightConstants {

		// declare ID's of pipelines here
		public static final int kCubePipeline = 0;
		public static final int kReflectivePipeline = 1;
		public static final int kApriltagPipeline = 2;

		// Servo Constants
		public static final int kServoPort = 2;
		// sets the offset of the sevo so the limelight is facing forward (double 0 - 1)
		public static final double kServoFrontPose = .76;
		public static final double kServoBackpose = 0;

		// piss values for limelight
		public static final PIDGains kLLTargetGains = new PIDGains(0.008, 0, 0);

		public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0); // .008
		public static final PIDGains kLLPuppyDriveGains = new PIDGains(0.008, 0, 0);
		public static final double kPuppyTurnMotionSmoothing = 0.3;
		public static final double kPuppyDriveMotionSmoothing = 0.4;

		public static final PIDGains kLLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
		public static final PIDGains kLLAlignDriveGains = new PIDGains(.025, 0.0015, 0.0005);
		public static final double kAlignDriveMotionSmoothing = 0;
		public static final double kAlignStrafeMotionSmoothing = 0;

	}

	public static final String kRobotName = "Final Competition Bot";

	public static final String kRioCANBusName = "rio";

	public static final String kCTRECANBusName = "rio";

}