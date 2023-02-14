package frc.robot.Utils;

import frc.robot.Constants.OperatorConstants;

public class JoystickUtils {

	/**
	 * Applies a deadband to the input value from OperatorConstants
	 * 
	 * @param input the value to be constrained
	 * @return the constrained value
	 */
	public static double deadBand(double input) {
		if (Math.abs(input) < OperatorConstants.KDeadBand) {
			return 0;
		}
		return input;
	}

	/**
	 * squares the joystick input and uses clever math to compansate for the offset
	 * caused by the deadband
	 * 
	 * @param input The input from the joystick
	 * @return The corrected joystick values
	 */
	public static double processJoystickInput(double input) {

		if (deadBand(input) == 0)
			return 0;

		double correctedValue = input;

		correctedValue = (correctedValue - (OperatorConstants.KDeadBand * Math.signum(correctedValue)))
				/ (1 - OperatorConstants.KDeadBand);

		correctedValue = Math.copySign(Math.pow(correctedValue, 3), input);

		return correctedValue;
	}

}