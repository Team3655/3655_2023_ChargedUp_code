package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

	/** controls when the arm is tucked */
	private boolean isTucked;

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	/** the variable setting the height of the arm */
	ArmPoses armState;

	/** used for controling the height of the arm */
	private enum ArmPoses {
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_INTAKE,
		MID_INTAKE,
		HIGH_INTAKE
	}

	public ArmSubsystem() {
		isTucked = true;
		isFront = true;
		armState = ArmPoses.LOW_INTAKE;
	}

	public CommandBase exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
				});
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// region setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param pos can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE, MID_INTAKE,
	 *            HIGH_INTAKE)
	 */
	public void setArmState(ArmPoses pos) {
		armState = pos;
	}

	/**
	 * Sets the dominant side of te robot
	 * 
	 * @param side if true the front will be dominant
	 */
	public void setFront(boolean side) {
		isFront = side;
	}

	/**
	 * Toggles the dominant side of the robot
	 */
	public void setToggleSide() {
		isFront = !isFront;
	}

	/**
	 * sets the robot to be tucked
	 * 
	 * @param tuck if true the robot will tuck itself
	 */
	public void setTucked(boolean tuck) {
		isTucked = tuck;
	}

	// endregion

}
