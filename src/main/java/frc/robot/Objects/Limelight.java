/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Objects;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
	NetworkTable limelight; // Table for the limelight
	NetworkTableEntry tx; // Table for the x-coordinate
	NetworkTableEntry ty; // Table for the y-coordinate
	NetworkTableEntry ta; // Table for the area
	NetworkTableEntry ts; // Table for the skew
	NetworkTableEntry tv; // Table to see if there are valid targets
	NetworkTableEntry tl; // Table for latency
	NetworkTableEntry tshort; // Table for short side length
	NetworkTableEntry tlong; // Table for long side length
	NetworkTableEntry thoriz; // Table for width
	NetworkTableEntry tvert; // Table for height
	NetworkTableEntry ledMode; // Table to set blinking leds
	NetworkTableEntry camMode; // Table to set camera mode
	NetworkTableEntry pipeline; // Table to switch pipelines
	NetworkTableEntry solvePNP;
	double[] defaultArray = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	public void GetLimelightValues() {

		limelight = NetworkTableInstance.getDefault().getTable("limelight");// Instantiate the tables
		tx = limelight.getEntry("tx");
		ty = limelight.getEntry("ty");
		ta = limelight.getEntry("ta");
		ts = limelight.getEntry("ts");
		tv = limelight.getEntry("tv");
		tl = limelight.getEntry("tl");
		tshort = limelight.getEntry("tshort");
		tlong = limelight.getEntry("tlong");
		thoriz = limelight.getEntry("thoriz");
		tvert = limelight.getEntry("tvert");
		ledMode = limelight.getEntry("ledMode");
		camMode = limelight.getEntry("camMode");
		pipeline = limelight.getEntry("pipeline");
		solvePNP = limelight.getEntry("camtran");

		SmartDashboard.putNumber("LimeLightX", tx.getDouble(getArea()));
		SmartDashboard.putNumber("LimeLightY", ty.getDouble(getArea()));
		SmartDashboard.putNumber("LimelightArea", getArea());
		SmartDashboard.putNumber("Pipeline", getPipeline());
		SmartDashboard.putBoolean("ValitTarget", hasValidTarget());
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the distance
	 * from the target in inches. The limelight must be in Solve3D mode with
	 * High-Res enabled.
	 * 
	 * @return Distance from the target
	 */
	public double getDistance() {
		return Math.sqrt(Math.pow(getXPos(), 2) + Math.pow(getYPos(), 2));
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the x-distance
	 * from the target in inches. The limelight must be in Solve3D mode with
	 * High-Res enabled.
	 * 
	 * @return x-distance from the target in inches
	 */
	public double getXPos() {
		return solvePNP.getDoubleArray(defaultArray)[0];
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the y-distance
	 * from the target in inches. The limelight must be in Solve3D mode with
	 * High-Res enabled.
	 * 
	 * @return y-distance from the target in inches
	 */
	public double getYPos() {
		return solvePNP.getDoubleArray(defaultArray)[1];
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the z-distance
	 * from the target in inches. The limelight must be in Solve3D mode with
	 * High-Res enabled.
	 * 
	 * @return y-distance from the target in inches
	 */
	public double getZPos() {
		return solvePNP.getDoubleArray(defaultArray)[2];
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the pitch of
	 * the robot. The limelight must be in Solve3D mode with High-Res enabled.
	 * 
	 * @return Pitch of the robot in degrees
	 */
	public double getPitch() {
		return solvePNP.getDoubleArray(defaultArray)[3];
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the yaw of the
	 * robot. The limelight must be in Solve3D mode with High-Res enabled.
	 * 
	 * @return Yaw of the robot in degrees
	 */
	public double getYaw() {
		return solvePNP.getDoubleArray(defaultArray)[4];
	}

	/**
	 * This function uses the Limelight's Solve3D function to compute the roll of
	 * the robot. The limelight must be in Solve3D mode with High-Res enabled.
	 * 
	 * @return Roll of the robot in degrees
	 */
	public double getRoll() {
		return solvePNP.getDoubleArray(defaultArray)[5];
	}

	/**
	 * This function returns the target's side-to-side angle from the center of the
	 * Limelight's field of view
	 * 
	 * @return x-angle of the target in degrees
	 */
	public double getX() {
		return tx.getDouble(0.0);
	}

	/**
	 * This function returns the target's up-down angle from the center of the
	 * Limelight's field of view
	 * 
	 * @return y-angle of the target in degrees
	 */
	public double getY() {
		return ty.getDouble(0.0);
	}

	/**
	 * This function returns the target's area in the image
	 * 
	 * @return Area of the target in pixels
	 */
	public double getArea() {
		return ta.getDouble(0.0);
	}

	/**
	 * This function returns the skew of the target in relation to the robot
	 * 
	 * @return Skew of the target in degrees
	 */
	public double getSkew() {
		return ts.getDouble(0.0);
	}

	/**
	 * This function returns if there is a valid vision target
	 * 
	 * @return True if there is a target
	 */
	public boolean hasValidTarget() {
		return tv.getDouble(0) == 1.0;
	}

	/**
	 * This function returns the processing latency of the Limelight
	 * 
	 * @return Milliseconds of delay
	 */
	public double getLatency() {
		return tl.getDouble(0.0);
	}

	/**
	 * This function returns the height of the shorter side of the target (works
	 * well on targets like Stronghold)
	 * 
	 * @return Height of short side in pixels
	 */
	public double getShortSide() {
		return tshort.getDouble(0.0);
	}

	/**
	 * This function returns the height of the longer side of the target (works well
	 * on targets like Stronghold)
	 * 
	 * @return Height of long side in pixels
	 */
	public double getLongSide() {
		return tlong.getDouble(0.0);
	}

	/**
	 * This function returns the width of the target
	 * 
	 * @return Width of the target in pixels
	 */
	public double getWidth() {
		return thoriz.getDouble(0.0);
	}

	/**
	 * This function returns the height of the target
	 * 
	 * @return Height of the target in pixels
	 */
	public double getHeight() {
		return tvert.getDouble(0.0);
	}

	/**
	 * This function sets the pipeline ID for when you have multiple different
	 * pipelines
	 * 
	 * @param ID
	 */
	public void setPipeline(int id) {
		pipeline.setNumber(id);
	}

	public Integer getPipeline() {
		NetworkTableEntry pipeline = limelight.getEntry("pipeline");
		Integer pipe = (int) pipeline.getDouble(0.0);
		return pipe;
	}

	/**
	 * Set the state of the LEDs
	 * 
	 * @param mode
	 *             0- Pipeline default
	 *             1- Force off
	 *             2- Force blink
	 *             3- Force on
	 */
	public void setLedMode(int mode) {
		ledMode.setNumber(mode);
	}

	public void setCamMode(int mode) {
		camMode.setNumber(mode);
	}
}