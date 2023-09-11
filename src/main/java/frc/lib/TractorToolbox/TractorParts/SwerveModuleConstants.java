package frc.lib.TractorToolbox.TractorParts;

public class SwerveModuleConstants {

	// public final int kCANCoderID;
	public final int kTurnMotorID;
	public final int kLeaderDriveMotorID;
	public final int kFolloweDriveMotorID;
	public final double kAngleZeroOffset;

	public SwerveModuleConstants(
			// int CANCoderID,
			int TurnMotorID,
			int LeaderDriveMotorID,
			int FolloweDriveMotorID,
			double AngleZeroOffset) {
		// kCANCoderID = CANCoderID;
		kTurnMotorID = TurnMotorID;
		kLeaderDriveMotorID = LeaderDriveMotorID;
		kFolloweDriveMotorID = FolloweDriveMotorID;
		kAngleZeroOffset = AngleZeroOffset;
	}

}