package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Objects.ArmSegment;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	/** used for controling the height of the arm */
	public enum ArmPoses {
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

	/** the variable setting the height of the arm */
	ArmPoses m_armState, m_prevArmState;

	// create arms
	public ArmSegment m_majorArm;
	public ArmSegment m_minorArm;

	// endregion

	public ArmSubsystem() {
		// region: def arms

		// major arm defs
		m_majorArm = new ArmSegment(
				ArmConstants.kRightMajorArmPort,
				ArmConstants.kLeftMajorArmPort,
				ArmConstants.kMajorArmTicks,
				true);

		m_majorArm.setPID(
				ArmConstants.kMajorArmP,
				ArmConstants.kMajorArmI,
				ArmConstants.kMajorArmD);

		m_majorArm.setConstraints(-90, 90);

		// minor arm defs
		m_minorArm = new ArmSegment(
				ArmConstants.kRightMinorArmPort,
				ArmConstants.kLeftMinorArmPort,
				ArmConstants.kMinorArmTicks,
				false);

		m_minorArm.setPID(
				ArmConstants.kMinorArmP,
				ArmConstants.kMinorArmI,
				ArmConstants.kMinorArmD);

		m_minorArm.setConstraints(-90, 90);
		// endregion

		// the default state of the arms
		m_isFront = true;

		// setArmState(ArmPoses.TUCKED);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("major real theta: ", m_majorArm.getRealTheta());
		SmartDashboard.putNumber("minor real theta: ", m_minorArm.getRealTheta());
		// Address the major motors
		m_majorArm.setReference();
		// Address the minor motors
		m_minorArm.setReference();

	}

	// region setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param pose can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE, MID_INTAKE,
	 *             HIGH_INTAKE)
	 */
	public void setArmState(ArmPoses pose) {

		m_armState = pose;

		switch (m_armState) {
			// When the arms are tucked in the center of the robot (this is the only legal
			// starting position)
			case TUCKED:
				m_majorArm.setTargetTheta(0);
				m_minorArm.setTargetTheta(0);
				break;

			// Used for scoring in the lowest "hybrid" node
			case LOW_SCORE:
				m_majorArm.setTargetTheta(45);
				m_minorArm.setTargetTheta(90);
				break;

			// Used for scoring in the middle node
			case MID_SCORE:
				m_majorArm.setTargetTheta(75);
				m_minorArm.setTargetTheta(90);
				break;

			// Used for scoring in the highest node
			case HIGH_SCORE:
				m_majorArm.setTargetTheta(90);
				m_minorArm.setTargetTheta(90);
				break;

			// Used for intaking off of the floor
			case LOW_INTAKE:
				m_majorArm.setTargetTheta(30);
				m_minorArm.setTargetTheta(30);
				break;

			// Used for intaking from the human player chute
			case MID_INTAKE:
				m_majorArm.setTargetTheta(30);
				m_minorArm.setTargetTheta(45);
				break;

			// Used for intaking from the sliding human player station
			case HIGH_INTAKE:
				m_majorArm.setTargetTheta(80);
				m_minorArm.setTargetTheta(80);
				break;

			// goes to the pair of angles defined my the TSB driver
			case DRIVER_CONTROL:
				break;
		}

		// Address the major motors
		m_majorArm.setReference();
		// Address the minor motors
		m_minorArm.setReference();

	}

	/**
	 * Sets the previous height of the arm
	 * 
	 * @param pose can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE, MID_INTAKE,
	 *             HIGH_INTAKE)
	 */
	public void setPrevArmState(ArmPoses pose) {
		m_prevArmState = pose;
	}

	/**
	 * Sets the dominant side of te robot
	 * 
	 * @param isFront if true the front will be dominant
	 */
	public void setDominantSide(boolean isFront) {
		if (isFront) {
			m_majorArm.setSign(1);
			m_minorArm.setSign(1);
		} else {
			m_majorArm.setSign(-1);
			m_minorArm.setSign(-1);
		}
	}

	/** Toggles the dominant side of the robot */
	public CommandBase toggleSide() {
		m_isFront = !m_isFront;
		return runOnce(() -> {
			setDominantSide(m_isFront);
		});
	}

	// endregion

	// region getters

	/**
	 * Used to get the target height of the arm as an enum
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public ArmPoses getArmState() {
		return m_armState;
	}

	/**
	 * Used to get the previous target height of the arm as an enum
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public ArmPoses getPrevArmState() {
		return m_prevArmState;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return m_isFront;
	}

	public boolean getAtTarget(double deadBand) {
		if (m_majorArm.getAtTarget(deadBand) && m_minorArm.getAtTarget(deadBand)) {
			return true;
		}
		return false;
	}

	// endregion

}
