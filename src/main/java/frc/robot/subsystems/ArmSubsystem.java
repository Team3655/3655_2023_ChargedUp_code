package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.Mechanisms.ArmSegment;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	public HashMap<kArmPoses, double[]> armStates = ArmConstants.kArmStatesMap;

	/** used to track the state of the arm */
	private kArmPoses targetArmState;

	/** controls the side of the robot the arm is on */
	private boolean isFront;
	private boolean enableArms;

	// create arms
	private ArmSegment majorArm;
	private ArmSegment minorArm;

	// endregion

	public ArmSubsystem() {

		// this will cause the code to fail to run if the hashmap is not full
		for (kArmPoses pose : kArmPoses.values()) {
			try {
				double x = 0;
				x = x + armStates.get(pose)[0];
				x = x + armStates.get(pose)[1];
			} catch (Exception exception) {
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
				false);

		majorArm.setPID(ArmConstants.kMajorArmGains);

		majorArm.setConstraints(ArmConstants.kMajorArmConstraints);
		majorArm.setMaxOutput(ArmConstants.kMajorPIDOutputLimit);
		majorArm.setTrapazoidalConstraints(ArmConstants.kMaxMajorVelRadiansPerSec, ArmConstants.kMaxMajorAccelRadiansPerSec);

		// minor arm defs
		minorArm = new ArmSegment(
				ArmConstants.kRightMinorArmPort,
				ArmConstants.kLeftMinorArmPort,
				ArmConstants.kMinorArmTicks,
				true);

		minorArm.setPID(ArmConstants.kMinorArmGains);

		minorArm.setConstraints(ArmConstants.kMinorArmConstraints);
		minorArm.setMaxOutput(ArmConstants.kMinorPIDOutputLimit);
		minorArm.setTrapazoidalConstraints(ArmConstants.kMaxMinorVelRadiansPerSec, ArmConstants.kMaxMinorAccelRadiansPerSec);
		// endregion

		// the default state of the arms
		isFront = true;
		enableArms = true;

		setSequencedArmState(kArmPoses.TUCKED);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		SmartDashboard.putString("ArmState", targetArmState.toString());

		SmartDashboard.putNumber("major target", majorArm.getTargetTheta());
		SmartDashboard.putNumber("minor target", minorArm.getTargetTheta());

		SmartDashboard.putNumber("major real theta: ", majorArm.getRealTheta());
		SmartDashboard.putNumber("minor real theta: ", minorArm.getRealTheta());

		SmartDashboard.putNumber("major left real theta", majorArm.getLeftRealTheta());
		SmartDashboard.putNumber("major right real theta", majorArm.getRightRealTheta());

		SmartDashboard.putNumber("major left real theta", majorArm.getLeftRealTheta());
		SmartDashboard.putNumber("major right real theta", majorArm.getRightRealTheta());

		SmartDashboard.putNumber("major power draw: ", majorArm.getPowerDraw());
		SmartDashboard.putNumber("minor power draw: ", minorArm.getPowerDraw());

		SmartDashboard.putBoolean("At target: ", getAtTarget(8));
		SmartDashboard.putBoolean("At target major", majorArm.getAtTarget(5));
		SmartDashboard.putBoolean("At target minor", minorArm.getAtTarget(5));

		SmartDashboard.putNumber("LeftMajorOutput", majorArm.getLeftMotorOutput());
		SmartDashboard.putNumber("RightMajorOutput", majorArm.getRightMotorOutput());

		updateSequencing();
	}

	// region Commands

	public CommandBase UnsequencedArmPoseCommand(final kArmPoses state) {
		return runOnce(() -> {
			setUnsequencedArmState(state);
		});
	}

	public CommandBase SequencedArmPoseCommand(final kArmPoses state) {
		return runOnce(() -> {
			setSequencedArmState(state);
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
			enableArms = false;
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
	 *              LOW_INTAKE, MID_INTAKE, HIGH_INTAKE)
	 */
	public void setUnsequencedArmState(kArmPoses state) {
		setTargetArmState(state);
		majorArm.setReference();
		minorArm.setReference();
	}

	public void setSequencedArmState(kArmPoses state) {

		setTargetArmState(state);

		if (state == kArmPoses.TUCKED) {
			minorArm.setReference();
		} else {
			majorArm.setReference();
		}
	}

	/**
	 * @param targetArmState the targetArmState to set
	 */
	public void setTargetArmState(kArmPoses state) {
		targetArmState = state;
		enableArms = true;

		// get minor speed from map
		// gets the angle values from the hashmap
		majorArm.setTargetTheta(armStates.get(targetArmState)[0]);
		minorArm.setTargetTheta(armStates.get(targetArmState)[1]);
	}

	public void updateSequencing() {
		if ((majorArm.getAtTarget(30) || minorArm.getAtTarget(30)) && enableArms) {
			majorArm.setReference();
			minorArm.setReference();
		}
	}

	// endregion

	// region getters

	/**
	 * Used to get the target height of the arm as an enum
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public kArmPoses getArmState() {
		return targetArmState;
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

	public double[] getTargetTheta() {
		return new double[] { majorArm.getTargetTheta(), minorArm.getTargetTheta() };
	}

	// endregion

}
