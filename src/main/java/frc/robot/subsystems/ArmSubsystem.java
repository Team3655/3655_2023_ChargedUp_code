package frc.robot.subsystems;

import java.util.HashMap;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Objects.ArmSegment;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {

	// region properties

	public HashMap<ArmPoses, double[]> armStates = ArmConstants.kArmStates;

	/** used to track the state of the arm */
	ArmPoses targetArmState;

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	// create arms
	private ArmSegment majorArm;
	private ArmSegment minorArm;

	// endregion

	public ArmSubsystem() {

		// this will cause the code to fail to run if the hashmap is not full
		for (ArmPoses pose : ArmPoses.values()) {
			try {
				double x = 0;
				x = x + armStates.get(pose)[0];
				x = x + armStates.get(pose)[1];
				x = x + armStates.get(pose)[2];
			} catch(Exception exception) {
				throw new IndexOutOfBoundsException(
				"NOT ALL ARM POSES HAVE A VALUE IN THE HASHMAP! THIS WILL RESLUT IN CRASHING IF NOT RESOLVED!");
			}
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
				ArmConstants.kMajorArmIzone);

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
				ArmConstants.kMinorArmIzone);

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
		SmartDashboard.putNumber("major power draw: ", majorArm.getPowerDraw());
		SmartDashboard.putNumber("minor power draw: ", minorArm.getPowerDraw());
		SmartDashboard.putBoolean("At target: ", getAtTarget(5));
		SmartDashboard.putBoolean("At target major", majorArm.getAtTarget(5));
		SmartDashboard.putBoolean("At target major", majorArm.getAtTarget(5));



	}

	// region Commands

	public CommandBase ArmPoseCommand(final ArmPoses state) {
		return runOnce(() -> {
			setArmState(state);
		});
	}

	/** Toggles the dominant side of the robot */
	public void ToggleSide() {
			isFront = !isFront;
			majorArm.setSign((isFront) ? 1 : -1);
			minorArm.setSign((isFront) ? 1 : -1);
			majorArm.setReference();
			minorArm.setReference();
	}

	public CommandBase toggleArmMotors() {
		return runOnce(() -> {
			minorArm.toggleMotors();
			majorArm.toggleMotors();
		});
	}

	public CommandBase zeroArms() {
		return runOnce(() -> {
			minorArm.resetZeros();
			majorArm.resetZeros();
		});
	}

	// endregion 

	// region Setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param state can be (LOW_SCORE, MID_SCORE, HIGH_SCORE,
	 *             LOW_INTAKE, MID_INTAKE, HIGH_INTAKE)
	 */
	public void setArmState(final ArmPoses state) {
		targetArmState = state;

		// gets the angle values from the hashmap
		majorArm.setTargetTheta(armStates.get(targetArmState)[0]);
		minorArm.setTargetTheta(armStates.get(targetArmState)[1]);

		minorArm.setConstraints(ArmConstants.kMinorArmConstraints, armStates.get(targetArmState)[2]);

		majorArm.setReference();
		minorArm.setReference();

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
		return targetArmState;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	public boolean getAtTarget(final double deadBand) {

		if (majorArm.getAtTarget(deadBand) && minorArm.getAtTarget(deadBand)) {
			return true;
		}
		return false;
	}

	public double[] getTargetTheta() {
		return new double[]{majorArm.getTargetTheta(), minorArm.getTargetTheta()};
	}

	// endregion

}
