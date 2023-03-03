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
import frc.robot.commands.Autonomous.IntakeSequence;
import frc.robot.commands.Autonomous.ScoreSequence;

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
		public static final int kTurnMotorCurrentLimit = 15;
		public static final int kDriveMotorCurrentLimit = 25;

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);

		// Constants set for the _SDS MK4i_
		public static final double kdriveGearRatio = 1d / 6.75;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// TODO: Set feedforward values for turning
		public static final double ksVolts = .1;
		public static final double kDriveFeedForward = .2;

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
		public static final double kFrontLeftAngleZero = 157.412;
		public static final double kFrontRightAngleZero = 76.025;
		public static final double kRearLeftAngleZero = -30.761;
		public static final double kRearRightAngleZero = -23.642;

		public static final double kModuleDriveControllerP = .1;
		public static final double kModuleDriveControllerI = 0;
		public static final double kModuleDriveControllerD = 0;

		public static final double kModuleTurningControllerP = 6.5;
		public static final double kModuleTurningControllerI = 0.25;
		public static final double kModuleTurningControllerD = 0.15;

		// SPARK MAX Angular PID values
		public static double[] kAngularPID = {
				kModuleTurningControllerP,
				kModuleTurningControllerI,
				kModuleTurningControllerD };

		// SPARK MAX Drive PID values
		public static double[] kDrivePID = {
				kModuleDriveControllerP,
				kModuleDriveControllerI,
				kModuleDriveControllerD };
	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.5;
		public static final double kMaxSpeedMetersPerSecond = 3.0;
		public static final double kMaxTurboMetersPerSecond = 4.5;

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
			public static final PIDGains kPPDriveGains = new PIDGains(.75, 0, 0);

			public static final PIDGains kPPTurnGains = new PIDGains(1, 0, 0);

			public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
				{
					put("ScoreHigh", new ScoreSequence(ArmPoses.HIGH_SCORE));
					put("ScoreMid", new ScoreSequence(ArmPoses.MID_SCORE));
					put("ScoreLow", new ScoreSequence(ArmPoses.LOW_SCORE));
					put("IntakeDown", new IntakeSequence());
					put("DropArm", new ArmPoseCommand(ArmPoses.LOW_INTAKE));
				}
			};
		}

		public static final PIDGains kTurnCommandGains = new PIDGains(.02, 0, 0);
		public static final double kTurnCommandMaxVelocity = 1;
		public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 1;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = 2;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.01, 0, 0);
		public static final double kMaxBalancingVelocity = 10;
		public static final double kMaxBalancingAcceleration = 5;

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

		public static final int kLeftGripperPort = 1;
		public static final int kRightGripperPort = 2 ;

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
		public static final int kMajorArmCurrentLimit = 20;
		public static final int kMinorArmCurrentLimit = 15;

		// speed limits for the arms
		public static final double kMajorArmPIDOutputLimit = .35;
		public static final double kMinorArmPIDOutputLimit = .4;

		// angle limits for the arms (min will be set to -input)
		public static final double kMajorArmConstraints = 101;
		public static final double kMinorArmConstraints = 180;

		// Arm PID constants
		public static final double kMajorArmP = 3;
		public static final double kMajorArmI = 0;
		public static final double kMajorArmD = 0;
		public static final double kMajorArmIzone = 5;

		public static final double kMinorArmP = 6;
		public static final double kMinorArmI = 0.0001;
		public static final double kMinorArmD = 0;
		public static final double kMinorArmIzone = 5;

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
				put(ArmPoses.TUCKED, new double[] { 0, 0, .25 });
				put(ArmPoses.LOW_SCORE, new double[] { 0, 90, kMinorArmPIDOutputLimit });
				put(ArmPoses.MID_SCORE, new double[] { 55, 33, .15 });
				put(ArmPoses.HIGH_SCORE, new double[] { 100, 55, .08 });
				put(ArmPoses.LOW_INTAKE, new double[] { -10, 98, kMinorArmPIDOutputLimit });
				put(ArmPoses.MID_INTAKE, new double[] { 13, 33, .25 });
				put(ArmPoses.HIGH_INTAKE, new double[] { 105, 80, .25 });
				put(ArmPoses.DRIVER_CONTROL, new double[] { 0, 0, kMinorArmPIDOutputLimit });
			}

		};

	}

	public static class IntakeConstants {

		// NEO 550 Sucker motor CAN ID's
		public static final int kSideSuckerPort = 17;
		public static final int kMainSuckerPort = 18;

		public static final int kSideSuckerCurrentLimit = 8;
		public static final int kMainSuckerCurrentLimit = 10;
		public static final int kMainSuckerStallCurrentLimit = 12;

		public static final double kMainSuckerSetpoint = .15;
		public static final int kHasPieceThreshold = 1000;

		public static final double kMainSuckerP = .009;

		public static final double kMainSuckerMaxOutput = 1d / 3d;

	}

	public static class LimelightConstants {

		// declare ID's of pipelines here
		public static final int kRetroReflectivePipeline = 0;
		public static final int kApriltagPipeline = 1;

		// Servo Constants
		public static final int kServoPort = 0;
		// sets the offset of the sevo so the limelight is facing forward (double 0 - 1)
		public static final double kServoFrontPose = .76;
		public static final double kServoBackpose = 0;

		// piss values for limelight
		public static final PIDGains LLTargetGains = new PIDGains(0.008, 0, 0); // TODO: gains for target command

		public static final PIDGains LLPuppyTurnGains = new PIDGains(0.008, 0, 0); // TODO: gains for puppydog command
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
