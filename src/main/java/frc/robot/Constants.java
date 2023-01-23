// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

		public static final double kPModuleDriveController = 0; // TODO: Set PID constants
		public static final double kPModuleTurningController = 0;
		public static final double kDModuleTurningController = 0;
		public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0;
		public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 0;

		// Constants set for the _SDS MK4i_
		public static final double kdriveGearRatio = 1 / 6.75;
		public static final double kturnGearRatio = 150 / 7; // TODO: Check steering ratio //TODO: Double check ratio
		public static final double kwheelCircumference = Math.PI * 0.1524; // 6" to meters

	}

	public static class DriveConstants {
		// TODO: Set feedforward values for drive
		public static final double ksVolts = 0;
		public static final double kvVoltSecondsPerMeter = 0;

		public static final double ksTurning = 0;
		public static final double kvTurning = 0;

		// TODO: Set CAN ID's for motors and CANcoders
		// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 0;
		public static final int kRearLeftDriveMotorPort = 0;
		public static final int kFrontRightDriveMotorPort = 0;
		public static final int kRearRightDriveMotorPort = 0;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 0;
		public static final int kRearLeftTurningMotorPort = 0;
		public static final int kFrontRightTurningMotorPort = 0;
		public static final int kRearRightTurningMotorPort = 0;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 3;
		public static final int kRearLeftTurningEncoderPort = 6;
		public static final int kFrontRightTurningEncoderPort = 9;
		public static final int kRearRightTurningEncoderPort = 12;

		// TODO: Set angle offset for CANcoders
		// Offset angle for absolute encoders (find this using REV client)
		public static final double kFrontLeftAngleZero = 0;
		public static final double kRearLeftAngleZero = 0;
		public static final double kFrontRightAngleZero = 0;
		public static final double kRearRightAngleZero = 0;

		// TODO: Set CAN ID for pigeon 2
		public static final int kPigeonPort = 0;

		// TODO: Set trackwidth and wheelbase in meters of physical robot
		public static final double kTrackWidth = 0; // meters
		public static final double kWheelBase = 0; // meters

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// TODO: Change max speed
		public static final double kMaxSpeedMetersPerSecond = 3;

		// TODO: Is gyro reversed?
		public static final boolean kGyroReversed = false;

	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	/**
	 * The constants pertaining to Arm (and sub arms)
	 */
	public static class ArmConstants {

		public static final int kMajorArmGearBoxRatio = 100;
		public static final int kMinorArmGearBoxRatio = 100;

		public static final int kMajorArmBeltRatio = 2 / 1;
		public static final int kMinorArmBeltRatio = 1;

		/**
		 * the total number of ticks for one 360 degree rotation of the arm
		 */
		public static final int kMajorArmTicks = MotorConstants.kNeoTicks * kMajorArmGearBoxRatio * kMajorArmBeltRatio;
		public static final int kMinorArmTicks = MotorConstants.kNeoTicks * kMinorArmGearBoxRatio * kMinorArmBeltRatio;

		/**
		 * The radius of each arms rotation in inches (from center of rotation to next
		 * arms center of rotation)
		 */
		public static final int kMajorArmLength = 38;
		public static final int kMinorArmLength = 23;

	}

	/**
	 * The constants pertaining to the types of motors shared between subsystems
	 */
	public static class MotorConstants {

		public static final int kNeoTicks = 42;

	}
}
