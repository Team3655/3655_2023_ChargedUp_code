// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.TractorToolbox.TractorParts.PIDGains;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.commands.ArmSwitchCommand;
import frc.robot.commands.Autonomous.IntakeDownSequence;
import frc.robot.commands.Autonomous.ScoreSequence;
import frc.robot.commands.Autonomous.SuckCommand;

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

		// Current limits for the wheels
		public static final int kTurnMotorCurrentLimit = 25;
		public static final int kDriveMotorCurrentLimit = 35;

		// Constants set for the _SDS MK4i_
		public static final double kdriveGearRatio = 1d / 6.75;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);

		public static final double ksVolts = .1;
		public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; // Tuned February 2, 2023

		// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 1;
		public static final int kFrontRightDriveMotorPort = 2;
		public static final int kRearLeftDriveMotorPort = 3;
		public static final int kRearRightDriveMotorPort = 4;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 5;
		public static final int kFrontRightTurningMotorPort = 6;
		public static final int kRearLeftTurningMotorPort = 7;
		public static final int kRearRightTurningMotorPort = 8;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 9;
		public static final int kFrontRightTurningEncoderPort = 10;
		public static final int kRearLeftTurningEncoderPort = 11;
		public static final int kRearRightTurningEncoderPort = 12;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = 34.805;
		public static final double kFrontRightAngleZero = 121.113;
		public static final double kRearLeftAngleZero = 149.414;
		public static final double kRearRightAngleZero = 61.699;

		public static final PIDGains kModuleDriveGains = new PIDGains(.1, 0, 0);

		public static final PIDGains kModuleTurningGains = new PIDGains(6.5, .25, .15);
	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.0;
		public static final double kMaxSpeedMetersPerSecond = 2.5;
		public static final double kMaxTurboMetersPerSecond = 4.5;

		// this sets turning speed (keep this low)
		public static final double kMaxRPM = 10;

		public static final int kPigeonPort = 20;

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
		public static final boolean kGyroTuring = false;

		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			public static final PIDGains kPPDriveGains = new PIDGains(4, 0, 0);
			public static final PIDGains kPPTurnGains = new PIDGains(5, 0, 0);

			public static final double kPPMaxVelocity = 3.0;
			public static final double kPPMaxAcceleration = 2.0;

			public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
				{
					put("Tuck", new ArmPoseCommand(ArmPoses.TUCKED));
					put("ScoreHigh", new ScoreSequence(ArmPoses.HIGH_SCORE));
					put("ScoreMid", new ScoreSequence(ArmPoses.MID_SCORE));
					put("ScoreLow", new ScoreSequence(ArmPoses.LOW_SCORE));
					put("IntakeDown", new IntakeDownSequence());
					put("ToggleSide", new ArmSwitchCommand());
					put("Suck", new SuckCommand(true, 250));
					put("Drop", new SuckCommand(false, 500));
				}
			};
		}

		public static final double kScoreSequenceDropTime = 3; // in seconds

		public static final PIDGains kTurnCommandGains = new PIDGains(.01, 0, 0);
		public static final double kTurnCommandMaxVelocity = 1;
		public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 1;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = 2;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.008, 0, 0);
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
		public static final int kRightMajorArmPort = 13;
		public static final int kLeftMajorArmPort = 14;
		public static final int kRightMinorArmPort = 15;
		public static final int kLeftMinorArmPort = 16;

		public static final int kLeftGripperPort = 0;
		public static final int kRightGripperPort = 1;

		public static final int kMajorArmGearBoxRatio = 100;
		public static final int kMinorArmGearBoxRatio = 100;

		public static final int kMajorArmBeltRatio = 2 / 1;
		public static final int kMinorArmBeltRatio = 1;

		/**
		 * the total number of motor rotations for one 360 degree rotation of the arm
		 */
		public static final int kMajorArmTicks = kMajorArmGearBoxRatio * kMajorArmBeltRatio;
		public static final int kMinorArmTicks = kMinorArmGearBoxRatio * kMinorArmBeltRatio;

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

		public static final double kMaxMajorVelRadiansPerSec = (Math.PI * 8) * 60;
		public static final double kMaxMajorAccelRadiansPerSec = (Math.PI * 6 * 60);

		
		public static final double kMaxMinorVelRadiansPerSec = (Math.PI * 8) * 60;
		public static final double kMaxMinorAccelRadiansPerSec = (Math.PI * 6 * 60);

		// angle limits for the arms
		public static final double kMajorArmConstraints = 101;
		public static final double kMinorArmConstraints = 180;

		// Arm PID constants
		public static final PIDGains kMajorArmGains = new PIDGains(0.0035, 0, 0.0005);

		public static final PIDGains kMinorArmGains = new PIDGains(0.0015, 0, 0.0005);

		public static enum ArmPoses {
			TUCKED,
			LOW_SCORE,
			MID_SCORE,
			HIGH_SCORE,
			LOW_INTAKE,
			MID_INTAKE,
			HIGH_INTAKE,
			DRIVER_CONTROL
		}

		public static final HashMap<ArmPoses, double[]> kArmStates = new HashMap<ArmPoses, double[]>() {
			{
				put(ArmPoses.TUCKED, new double[] { 0, 0 });
				put(ArmPoses.LOW_SCORE, new double[] { 0, 90 });
				put(ArmPoses.MID_SCORE, new double[] { 45, 28 });
				put(ArmPoses.HIGH_SCORE, new double[] { 100, 55 });
				put(ArmPoses.LOW_INTAKE, new double[] { -10, 98 });
				put(ArmPoses.MID_INTAKE, new double[] { 13, 33 });
				put(ArmPoses.HIGH_INTAKE, new double[] { 95, 80 });
				put(ArmPoses.DRIVER_CONTROL, new double[] { 0, 0 });
			}

		};

	}

	public static class IntakeConstants {

		public static final int kPnemnaticHubPort = 50;

		// NEO Sucker motor CAN ID's
		public static final int kSideSuckerPort = 17;
		public static final int kCenterSuckerPort = 18;

		public static final int kSideSolenoidPort = 1;
		public static final int kCenterSolenoidPort = 0;

		public static final int kCenterSuckerCurrentLimit = 8;
		public static final int kSideSuckerCurrentLimit = 5;

		public static final double kCenterSuckerSetpoint = 0.5;
		public static final int kHasPieceThreshold = 1000;

	}

	public static class LimelightConstants {

		// declare ID's of pipelines here
		public static final int kRetroReflectivePipeline = 0;
		public static final int kApriltagPipeline = 1;

		// Servo Constants
		public static final int kServoPort = 2;
		// sets the offset of the sevo so the limelight is facing forward (double 0 - 1)
		public static final double kServoFrontPose = .76;
		public static final double kServoBackpose = 0;

		// piss values for limelight
		public static final PIDGains LLTargetGains = new PIDGains(0.008, 0, 0);

		public static final PIDGains LLPuppyTurnGains = new PIDGains(0.008, 0, 0);
		public static final PIDGains LLPuppyDriveGains = new PIDGains(0.008, 0, 0);
		public static final double PuppyTurnMotionSmoothing = 0.3;
		public static final double PuppyDriveMotionSmoothing = 0.4;

		public static final PIDGains LLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
		public static final PIDGains LLAlignDriveGains = new PIDGains(.8, 0.0003, 0.0005);
		public static final double AlignDriveMotionSmoothing = .35;
		public static final double AlignStrafeMotionSmoothing = .25;

	}

	public static final String kRioCANBusName = "rio";

	public static final String kCanivoreCANBusName = "canivore";

}
