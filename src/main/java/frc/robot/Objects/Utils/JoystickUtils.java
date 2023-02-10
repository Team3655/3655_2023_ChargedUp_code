package frc.robot.Objects.Utils;

import frc.robot.Constants.DriveConstants;

public class JoystickUtils {
	
	public static double deadBand(double x) {
		if (Math.abs(x) < DriveConstants.KDeadBand) {
			return 0;
		}
		return x;
	}

}
