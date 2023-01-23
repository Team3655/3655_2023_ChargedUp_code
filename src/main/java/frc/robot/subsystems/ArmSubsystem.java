package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

	/** controls when the arm is tucked */
	private boolean setTucked;

	/** controls the side of the robot the arm is on */
	private boolean setFront;

	/** the variable setting the height of the arm */
	ArmPos armState;

	/** used for controling the height of the arm */
	private enum ArmPos {
		INTAKE,
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_COLLECT,
		HIGH_COLLECT
	}

	public ArmSubsystem() {

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

}
