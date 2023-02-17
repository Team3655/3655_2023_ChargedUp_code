package frc.robot.subsystems;

import java.util.HashMap;

import frc.robot.Constants.ArmConstants;
import frc.robot.Objects.ArmSegment;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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

	private final HashMap<ArmPoses, double[]> armStates = new HashMap<ArmPoses, double[]>();

	/** used to track the state of the arm */
	ArmPoses armState;

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	// create arms
	public ArmSegment majorArm;
	public ArmSegment minorArm;

	// endregion

	public ArmSubsystem() {

		armStates.put(ArmPoses.TUCKED, new double[]{0, 0});
		armStates.put(ArmPoses.LOW_SCORE, new double[]{25, 95});
		armStates.put(ArmPoses.MID_SCORE, new double[]{75, 90});
		armStates.put(ArmPoses.HIGH_SCORE, new double[]{90, 90});
		armStates.put(ArmPoses.LOW_INTAKE, new double[]{10, 95});
		armStates.put(ArmPoses.MID_INTAKE, new double[]{30, 45});
		armStates.put(ArmPoses.HIGH_INTAKE, new double[]{80, 80});
		armStates.put(ArmPoses.DRIVER_CONTROL, new double[]{0, 0});

		// this will cause the code to fail to run if the
		if (armStates.size() < ArmPoses.values().length) {
			throw new IndexOutOfBoundsException("NOT ALL ARM POSES HAVE A VALUE IN THE HASHMAP! THIS WILL RESLUT IN CRASHING IF NOT RESOLVED!");
		}

		// region: def arms

		// major arm defs
		majorArm = new ArmSegment(
				ArmConstants.kRightMajorArmPort,
				ArmConstants.kLeftMajorArmPort,
				ArmConstants.kMajorArmTicks,
				true);

		majorArm.setPID(
				ArmConstants.kMajorArmP,
				ArmConstants.kMajorArmI,
				ArmConstants.kMajorArmD,
				ArmConstants.kMajorArmIzone,
				ArmConstants.kMajorArmFF);

		majorArm.setConstraints(
				ArmConstants.kMajorArmConstraints,
				ArmConstants.kMajorArmPIDOutputLimit);

		// minor arm defs
		minorArm = new ArmSegment(
				ArmConstants.kRightMinorArmPort,
				ArmConstants.kLeftMinorArmPort,
				ArmConstants.kMinorArmTicks,
				false);

		minorArm.setPID(
				ArmConstants.kMinorArmP,
				ArmConstants.kMinorArmI,
				ArmConstants.kMinorArmD,
				ArmConstants.kMinorArmIzone,
				ArmConstants.kMinorArmFF);

		minorArm.setConstraints(
				ArmConstants.kMinorArmConstraints,
				ArmConstants.kMinorArmPIDOutputLimit);
		// endregion

		// the default state of the arms
		isFront = true;

		setArmState(ArmPoses.TUCKED);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		SmartDashboard.putNumber("major real theta: ", majorArm.getRealTheta());
		SmartDashboard.putNumber("minor real theta: ", minorArm.getRealTheta());

		majorArm.setReference();
		minorArm.setReference();

	}

	// region setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param pose can be (LOW_SCORE, MID_SCORE, HIGH_SCORE,
	 *             LOW_INTAKE, MID_INTAKE, HIGH_INTAKE)
	 */
	public void setArmState(ArmPoses pose) {
		armState = pose;

		// gets the angle values from the hashmap
		majorArm.setTargetTheta(armStates.get(armState)[0]);
		minorArm.setTargetTheta(armStates.get(armState)[1]);

		// Address the arm motors
		majorArm.setReference();
		minorArm.setReference();
	}

	/** Toggles the dominant side of the robot */
	public CommandBase ToggleSide() {
		return runOnce(() -> {
			isFront = !isFront;
			majorArm.setSign((isFront) ? 1 : -1);
			minorArm.setSign((isFront) ? 1 : -1);
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
		return armState;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	public boolean getAtTarget(double deadBand) {
		if (majorArm.getAtTarget(deadBand) && minorArm.getAtTarget(deadBand)) {
			return true;
		}
		return false;
	}

	// endregion

}
