// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Objects.Limelight;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {

	private Limelight limelight;
	private Servo LimeLightFlipper;

	/** Creates a new LimelightSubsystem */
	public LimelightSubsystem() {
		limelight = new Limelight();
		LimeLightFlipper = new Servo(LimelightConstants.kServoPort);
		LimeLightFlipper.set(LimelightConstants.kServoFrontPose);
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public CommandBase FlipLimelight(boolean isFront) {
		return runOnce(
				() -> {
					if (isFront) {
						LimeLightFlipper.set(LimelightConstants.kServoFrontPose);
					} else {
						LimeLightFlipper.set(LimelightConstants.kServoBackpose);
					}
				});
	}

	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		limelight.updateAll();
	}

	/** Points the robots heading at the target by rotating the bot */
	public void aimAtTarget() {
		// TODO: finish method
	}

	/**
	 * points the robot at the target by locking the heading and translating the bot
	 * 
	 * @param angle The heading to lock the robot to
	 */
	public void alignWithTarget(double headingDegrees) {
		// TODO: finish method
	}

}
