// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardSubsystem extends SubsystemBase {

	//private static DriveSubsystem m_driveSubsystem;
	private final DriveSubsystem m_drive;

	public DashboardSubsystem(DriveSubsystem subsystem) {
		m_drive = subsystem;
	}

	@Override
	public void periodic() {
		ModuleTelemetry();
	}

	public static class PIDConstants {

		// PID Shuffleboard tab
		private static ShuffleboardTab m_PIDTab = Shuffleboard.getTab("PID");

		// Entries of PathPlanner PID Values
		private static GenericEntry m_PathPlanner_X_kP = m_PIDTab.addPersistent("PathPlanner X kP", 0).getEntry();

		private static GenericEntry m_PathPlanner_X_kd = m_PIDTab.addPersistent("PathPlanner X kD", 0).getEntry();

		private static GenericEntry m_PathPlanner_Y_kP = m_PIDTab.addPersistent("PathPlanner Y kP", 0).getEntry();

		private static GenericEntry m_PathPLanner_Y_kD = m_PIDTab.addPersistent("PathPLanner Y kD", 0).getEntry();

		private static GenericEntry m_PathPlanner_rot_kP = m_PIDTab.addPersistent("PathPlanner Rot kP", 0).getEntry();

		private static GenericEntry m_PathPlanner_rot_kD = m_PIDTab.addPersistent("PathPlanner Rot kD", 0).getEntry();

		public static double getPlanner_X_kP() {
			return m_PathPlanner_X_kP.getDouble(0);
		}

		public static double getPlanner_X_kD() {
			return m_PathPlanner_X_kd.getDouble(0);
		}

		public static double getPlanner_Y_kP() {
			return m_PathPlanner_Y_kP.getDouble(0);
		}

		public static double getPlanner_Y_kD() {
			return m_PathPLanner_Y_kD.getDouble(0);
		}

		public static double getPlanner_Rot_kP() {
			return m_PathPlanner_rot_kP.getDouble(0);
		}

		public static double getPlanner_Rot_kD() {
			return m_PathPlanner_rot_kD.getDouble(0);
		}

		// Entries for Drive/Turn PID

		private static GenericEntry m_Drive_kP = m_PIDTab.addPersistent("Drive kP", 0).getEntry();

		private static GenericEntry m_Drive_kD = m_PIDTab.addPersistent("Drive kD", 0).getEntry();

		private static GenericEntry m_Turn_kP = m_PIDTab.addPersistent("Turn kP", 0).getEntry();

		private static GenericEntry m_Turn_kD = m_PIDTab.addPersistent("Turn kD", 0).getEntry();

		public static double getDrive_kP() {
			return m_Drive_kP.getDouble(0);
		}

		public static double getDrive_kD() {
			return m_Drive_kD.getDouble(0);
		}

		public static double getTurn_kP() {
			return m_Turn_kP.getDouble(0);
		}

		public static double getTurn_kD() {
			return m_Turn_kD.getDouble(0);
		}

		// Entries for Major Arm PID

		private static GenericEntry m_MajorArm_kP = m_PIDTab.addPersistent("Major Arm kP", 0).getEntry();

		private static GenericEntry m_MajorArm_kD = m_PIDTab.addPersistent("Major Arm kD", 0).getEntry();

		private static GenericEntry m_MinorArm_kP = m_PIDTab.addPersistent("Minor Arm kP", 0).getEntry();

		private static GenericEntry m_MinorArm_kD = m_PIDTab.addPersistent("Minor Arm kD", 0).getEntry();

		public static double getMajor_kP() {
			return m_MajorArm_kP.getDouble(0);
		}

		public static double getMajor_kD() {
			return m_MajorArm_kD.getDouble(0);
		}

		public static double getMinor_kP() {
			return m_MinorArm_kP.getDouble(0);
		}

		public static double getMinor_kD() {
			return m_MinorArm_kD.getDouble(0);
		}
	}

	
	private ShuffleboardTab m_ModuleTab = Shuffleboard.getTab("Modules");

	private GenericEntry m_FrontLeftHeading = m_ModuleTab.add("FL Heading", 0).getEntry();

	private GenericEntry m_RearLeftHeading = m_ModuleTab.add("RL Heading", 0).getEntry();

	private GenericEntry m_FrontRightHeading = m_ModuleTab.add("FR Heading", 0).getEntry();

	private GenericEntry m_RearRightHeading = m_ModuleTab.add("RR Heading", 0).getEntry();

	private GenericEntry m_gyro = m_ModuleTab.add("Gyro", 0).getEntry();

	public void ModuleTelemetry() {
		// m_FrontLeftHeading.setDouble(m_drive.getFrontLeftHeading());
		// m_RearLeftHeading.setDouble(m_drive.getRearLeftHeading());
		// m_FrontRightHeading.setDouble(m_drive.getFrontRightHeading());
		// m_RearRightHeading.setDouble(m_drive.getRearRightHeading());

		m_gyro.setDouble(m_drive.getHeading());
	}



	
}
