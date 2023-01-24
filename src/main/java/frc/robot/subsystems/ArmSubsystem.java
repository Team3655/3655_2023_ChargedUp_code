package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	/** used for controling the height of the arm */
	private enum ArmPoses {
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_INTAKE,
		MID_INTAKE,
		HIGH_INTAKE
	}

	/** controls when the arm is tucked */
	private boolean isTucked;

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	/** the variable setting the height of the arm */
	ArmPoses armState;

	/** sets the target angle for the major arm */
	float majorArmTargetTheta;
	/** sets the target angle for the minor arm */
	float minorArmTargetTheta;

	// endregion

	public ArmSubsystem() {
		isTucked = true;
		isFront = true;
		armState = ArmPoses.LOW_INTAKE;
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

	// region getters

	/**
	 * Used to get the target height of the arm as an enum (arm will not be at this
	 * height if
	 * tucked)
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public ArmPoses getArmState() {
		return armState;
	}

	/** ruturns true if the robots target state is tucked */
	public boolean getIsTucked() {
		return isTucked;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	/**
	 * Used for getting the number of ticks required to turn an angle
	 * 
	 * @param theta the angke you want to turn (in radians)
	 * @param ticks the number of ticks required to make one revolution
	 * @return the number of motor ticks required to turn theta
	 */
	public int getThetaToTicks(float theta, int ticks) {
		return (int) (theta * (float) ticks);
	}

	// endregion

}
