package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	/** used for controling the height of the arm */
	private enum ArmPoses {
		TUCKED,
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_INTAKE,
		MID_INTAKE,
		HIGH_INTAKE,
		DRIVER_CONTROL
	}

	/** controls the side of the robot the arm is on */
	private boolean isFront;
	private boolean prevIsFront;
	private boolean switchingSides;

	/** the variable setting the height of the arm */
	ArmPoses armState;
	ArmPoses prevArmState;

	// create arms
	ArmSegment majorArm;
	ArmSegment minorArm;

	// endregion

	public ArmSubsystem() {

		// the default state of the arms
		isFront = true;
		prevIsFront = isFront;
		switchingSides = false;

		armState = ArmPoses.TUCKED;
		prevArmState = armState;

		// region: def arms

		// major arm defs
		majorArm = new ArmSegment(
				ArmConstants.kRightMajorArmPort,
				ArmConstants.kLeftMajorArmPort,
				ArmConstants.kMajorArmTicks);

		majorArm.setPID(
				ArmConstants.kMajorArmP,
				ArmConstants.kMajorArmI,
				ArmConstants.kMajorArmD);

		// minor arm defs
		minorArm = new ArmSegment(
				ArmConstants.kRightMinorArmPort,
				ArmConstants.kLeftMinorArmPort,
				ArmConstants.kMinorArmTicks);

		majorArm.setPID(
				ArmConstants.kMinorArmP,
				ArmConstants.kMinorArmI,
				ArmConstants.kMinorArmD);
		// endregion
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// skips math if state has not changed
		if (prevArmState != armState) {
			switch (armState) {
				// When the arms are tucked in the center of the robot (this is the only legal
				// starting position)
				case TUCKED:
					majorArm.setTagetTheta(0);
					minorArm.setTagetTheta(0);
					break;

				// Used for scoring in the lowest "hybrid" node
				case LOW_SCORE:
					majorArm.setTagetTheta(45);
					minorArm.setTagetTheta(90);
					break;

				// Used for scoring in the middle node
				case MID_SCORE:
					majorArm.setTagetTheta(75);
					minorArm.setTagetTheta(90);
					break;

				// Used for scoring in the highest node
				case HIGH_SCORE:
					majorArm.setTagetTheta(90);
					minorArm.setTagetTheta(90);
					break;

				// Used for intaking off of the floor
				case LOW_INTAKE:
					majorArm.setTagetTheta(30);
					minorArm.setTagetTheta(100);
					break;

				// Used for intaking from the human player chute
				case MID_INTAKE:
					majorArm.setTagetTheta(30);
					minorArm.setTagetTheta(45);
					break;

				// Used for intaking from the sliding human player station
				case HIGH_INTAKE:
					majorArm.setTagetTheta(80);
					minorArm.setTagetTheta(80);
					break;

				// goes to the pair of angles defined my the TSB driver
				case DRIVER_CONTROL:

					break;
			}

			// Offset the minor arm based on the angle of the major arm (this makes the
			// minor arm reletive to the robot)
			// TODO: Re-enable if arms are no longer virtual four bar
			// minorArmTargetTheta += majorArmTargetTheta;
		}

		// Tells the robot to switch sides if the dominant side is a switch
		if (isFront != prevIsFront) {
			switchingSides = true;
		}

		// Swaps the sign of the target angle if the dominant side of the robot is back

		// Address the major motors
		majorArm.setReference();
		// Address the minor motors
		minorArm.setReference();

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
	public void toggleSide() {
		isFront = !isFront;
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

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	/**
	 * This method is used to get the number of revolutions the arm motor has made
	 * 
	 * @param isMajor setting this to false will return the count of the minor arm
	 * @return the number of revolutions as a double
	 */
	public double getRightEncoderTicks(boolean isMajor) {
		if (isMajor) {
			return rightMajorEncoder.getPosition();
		}
		return rightMinorEncoder.getPosition();
	}

	/**
	 * converts encoder count of a motor into and angle value
	 * 
	 * @param totalTicks the number of rotations required to make one rotation
	 * @param encoder    the encoder to get the rotations from
	 * @return the real angle of the arm
	 */
	public double getRotationsToAngle(double totalTicks, RelativeEncoder encoder) {
		double percentage = totalTicks / encoder.getPosition();
		double theta = percentage * (2 * Math.PI);
		return theta;
	}

	// endregion

}
