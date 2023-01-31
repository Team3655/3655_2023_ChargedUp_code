package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import java.util.ArrayList;

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
	private boolean m_isFront;
	private boolean m_prevIsFront;
	private boolean m_switchingSides;

	/** the variable setting the height of the arm */
	ArmPoses m_armState;
	ArmPoses m_prevArmState;

	// create arms
	ArmSegment m_majorArm;
	ArmSegment m_minorArm;

	// endregion

	public ArmSubsystem() {

		// the default state of the arms
		m_isFront = true;
		m_prevIsFront = m_isFront;
		m_switchingSides = false;

		m_armState = ArmPoses.TUCKED;
		m_prevArmState = m_armState;

		// region: def arms

		// major arm defs
		m_majorArm = new ArmSegment(
				ArmConstants.kRightMajorArmPort,
				ArmConstants.kLeftMajorArmPort,
				ArmConstants.kMajorArmTicks);

		m_majorArm.setPID(
				ArmConstants.kMajorArmP,
				ArmConstants.kMajorArmI,
				ArmConstants.kMajorArmD);

		// minor arm defs
		m_minorArm = new ArmSegment(
				ArmConstants.kRightMinorArmPort,
				ArmConstants.kLeftMinorArmPort,
				ArmConstants.kMinorArmTicks);

		m_minorArm.setPID(
				ArmConstants.kMinorArmP,
				ArmConstants.kMinorArmI,
				ArmConstants.kMinorArmD);
		// endregion
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// skips math if state has not changed
		if (m_prevArmState != m_armState) {
			switch (m_armState) {
				// When the arms are tucked in the center of the robot (this is the only legal
				// starting position)
				case TUCKED:
					m_majorArm.setTagetTheta(0);
					m_minorArm.setTagetTheta(0);
					break;

				// Used for scoring in the lowest "hybrid" node
				case LOW_SCORE:
					m_majorArm.setTagetTheta(45);
					m_minorArm.setTagetTheta(90);
					break;

				// Used for scoring in the middle node
				case MID_SCORE:
					m_majorArm.setTagetTheta(75);
					m_minorArm.setTagetTheta(90);
					break;

				// Used for scoring in the highest node
				case HIGH_SCORE:
					m_majorArm.setTagetTheta(90);
					m_minorArm.setTagetTheta(90);
					break;

				// Used for intaking off of the floor
				case LOW_INTAKE:
					m_majorArm.setTagetTheta(30);
					m_minorArm.setTagetTheta(100);
					break;

				// Used for intaking from the human player chute
				case MID_INTAKE:
					m_majorArm.setTagetTheta(30);
					m_minorArm.setTagetTheta(45);
					break;

				// Used for intaking from the sliding human player station
				case HIGH_INTAKE:
					m_majorArm.setTagetTheta(80);
					m_minorArm.setTagetTheta(80);
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

		// Swaps the sign of the target angle if the dominant side of the robot is back

		// Address the major motors
		m_majorArm.setReference();
		// Address the minor motors
		m_minorArm.setReference();

	}

	// region setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param pos can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE, MID_INTAKE,
	 *            HIGH_INTAKE)
	 */
	public void setArmState(ArmPoses pos) {
		m_armState = pos;
	}

	/**
	 * Sets the dominant side of te robot
	 * 
	 * @param side if true the front will be dominant
	 */
	public void setIsFront(boolean side) {
		m_isFront = side;
	}

	/** Toggles the dominant side of the robot */
	public void toggleSide() {
		m_isFront = !m_isFront;
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
		return m_armState;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return m_isFront;
	}

	// endregion

}
