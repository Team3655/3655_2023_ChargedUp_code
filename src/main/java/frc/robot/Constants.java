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

		public static final double kModuleDriveControllerP = .1;
		public static final double kModuleDriveControllerI = .0001;
		public static final double kModuleDriveControllerD = 0; // TODO: Set PID constants
		public static final double kModuleTurningControllerP = .1;
		public static final double kModuleTurningControllerI = .0001;
		public static final double kModuleTurningControllerD = 0;

		// Constants set for the _SDS MK4i_
		public static final double kdriveGearRatio = 1 / 6.75;
		public static final double kturnGearRatio = 1/(150 / 7); 
		public static final double kwheelCircumference = Math.PI * 0.1016; // 4" to meters

						// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 1;
		public static final int kFrontRightDriveMotorPort = 4;
		public static final int kRearLeftDriveMotorPort = 7;
		public static final int kRearRightDriveMotorPort = 10;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 2;
		public static final int kFrontRightTurningMotorPort = 5;
		public static final int kRearLeftTurningMotorPort = 8;
		public static final int kRearRightTurningMotorPort = 11;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 3;
		public static final int kFrontRightTurningEncoderPort = 9;
		public static final int kRearLeftTurningEncoderPort = 6;
		public static final int kRearRightTurningEncoderPort = 12;

		// TODO: Set angle offset for CANcoders
		// Offset angle for absolute encoders (find this using REV client)
		public static final double kFrontLeftAngleZero = 0;
		public static final double kFrontRightAngleZero = 0;
		public static final double kRearLeftAngleZero = 0;
		public static final double kRearRightAngleZero = 0;

		//SPARK MAX Angular PID values
		public static double[] kFrontLeftAngularPID = {.1, 0, .1};
		public static double[] kFrontRightAngularPID = {.1, 0, .1};
		public static double[] kRearLeftAngularPID = {.1, 0, .1};
		public static double[] kRearRightAngularPID = {.1, 0, .1};

		//SPARK MAX Drive PID values
		public static double[] kFrontLeftDrivePID = {.1, 0, .1};
		public static double[] kFrontRightDrivePID = {.1, 0, .1};
		public static double[] kRearLeftDrivePID = {.1, 0, .1};
		public static double[] kRearRightDrivePID = {.1, 0, .1};
	}

	public static class DriveConstants {
		// TODO: Set feedforward values for drive
		public static final double ksVolts = .1; 
		public static final double kvVoltSecondsPerMeter = .1;

		public static final double ksTurning = .17161;//Tuned February 2, 2023
		public static final double kvTurning = .43205;

		public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4*Math.PI;
		public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 35 * Math.PI;

		// TODO: Change max speed
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxRPM = (kMaxModuleAngularSpeedRadiansPerSecond * 60)
				/ (2 * Math.PI); // Convert rad/s to RPM

		// TODO: Set CAN ID for pigeon 2
		public static final int kPigeonPort = 13;

		// TODO: Set trackwidth and wheelbase in meters of physical robot
		public static final double kTrackWidth = 24 / 39.37; // meters
		public static final double kWheelBase = 24 / 39.37; // meters

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


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

		public static final int kMajorArmDir = -1;
		public static final int kMinorArmDir = 1;

		public static final int kMajorArmGearBoxRatio = 100;
		public static final int kMinorArmGearBoxRatio = 100;

		public static final int kMajorArmBeltRatio = 2 / 1;
		public static final int kMinorArmBeltRatio = 1;

		/**
		 * the total number of ticks for one 360 degree rotation of the arm
		 */
		public static final int kMajorArmTicks = kMajorArmGearBoxRatio * kMajorArmBeltRatio;
		public static final int kMinorArmTicks = kMinorArmGearBoxRatio * kMinorArmBeltRatio;

		/**
		 * The radius of each arms rotation in inches (from center of rotation to next
		 * arms center of rotation)
		 */
		public static final int kMajorArmLength = 38;
		public static final int kMinorArmLength = 23;

		// NEO turning motor CAN ID's
		public static final int kRightMajorArmPort = 101;
		public static final int kLeftMajorArmPort = 102;
		public static final int kRightMinorArmPort = 103;
		public static final int kLeftMinorArmPort = 104;

		// Arm PID constants
		public static final int kMajorArmP = 0;
		public static final int kMajorArmI = 0;
		public static final int kMajorArmD = 0;
		public static final int kMinorArmP = 0;
		public static final int kMinorArmI = 0;
		public static final int kMinorArmD = 0;

	}

	/**
	 * The constants pertaining to the types of motors shared between subsystems
	 */
	public static class MotorConstants {

		public static final int kNeoTicks = 42;

	}
}
