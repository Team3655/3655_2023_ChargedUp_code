package frc.lib.TractorToolbox.TractorParts;

public class SwerveConstants {

	public final int kTurnSmartCurrentLimit;
	public final int kDriveSmartCurrentLimit;

	public final double kTurnGearRatio;
	public final double kDriveGearRatio;
	public final double kWheelCircumference;

	public final double kMaxModuleAccelMetersPerSecond;
	public final double kMaxModuleSpeedMetersPerSecond;

	public final double kDriveFeedForward;

	public SwerveConstants(
			int turnSmartCurrentLimit,
			int driveSmartCurrentLimit,
			double turnGearRatio,
			double driveGearRatio,
			double wheelCircumference,
			double maxModuleAccelMetersPerSecond,
			double maxModuleSpeedMetersPerSecond,
			double driveFeedForward) {

		this.kTurnSmartCurrentLimit = turnSmartCurrentLimit;
		this.kDriveSmartCurrentLimit = driveSmartCurrentLimit;
		this.kTurnGearRatio = turnGearRatio;
		this.kDriveGearRatio = driveGearRatio;
		this.kWheelCircumference = wheelCircumference;
		this.kMaxModuleAccelMetersPerSecond = maxModuleAccelMetersPerSecond;
		this.kMaxModuleSpeedMetersPerSecond = maxModuleSpeedMetersPerSecond;
		this.kDriveFeedForward = driveFeedForward;
	}
}
