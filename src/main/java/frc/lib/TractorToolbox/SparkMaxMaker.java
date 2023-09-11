package frc.lib.TractorToolbox;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxMaker {

	/**
	 * Creates a CANSparkMax object and restores its factory defaults (using this
	 * ensures the motors defaults are always restored)
	 * 
	 * @param motorID The CAN ID of the motor
	 * 
	 * @return A CANSparkMax object with a factory defaults restored
	 */
	public static CANSparkMax createSparkMax(int motorID) {

		CANSparkMax motor = new CANSparkMax(motorID, MotorType.kBrushless);

		motor.restoreFactoryDefaults();

		return motor;
	}

	/**
	 * Simplifies the creation of a Rev RelativeEncoder
	 * 
	 * @param motor     The CANSparkMax object to create the RelativeEncoder from
	 * @param unit      The unit of measurement being used (i.e. 360 for degrees,
	 *                  2pi for radians, etc.)
	 * @param gearRatio The gear ratio being applied to the motor
	 * 
	 * @return A RelativeEncoder with conversion factors based on the inputs
	 */
	public static RelativeEncoder createEncoder(CANSparkMax motor, double unit, double gearRatio) {

		RelativeEncoder encoder = motor.getEncoder();

		encoder.setPositionConversionFactor(unit * gearRatio);
		encoder.setVelocityConversionFactor(unit * gearRatio * (1d / 60d));

		return encoder;
	}

	/**
	 * Sends the telemetry of the motor and its internal encoder to the
	 * SmartDashboard (this is used to
	 * streamline moter telemetry, and ensure all relevent data is collected)
	 * 
	 * @param motorName The piece of text that will appear before the type of
	 *                  telemetry on SmartDashboard
	 * @param motor     The motor that telemetry will be read from. No need to
	 *                  include the encoder, it will be gotten directly from the
	 *                  moter
	 */
	public static void sendMoterTelemetry(String motorName, CANSparkMax motor) {

		RelativeEncoder encoder = motor.getEncoder();

		// Encoder Telemetry
		SmartDashboard.putNumber(motorName + " Position", encoder.getPosition());
		SmartDashboard.putNumber(motorName + " Velocity", encoder.getVelocity());
		SmartDashboard.putNumber(motorName + " PositionConversionFactor", encoder.getPositionConversionFactor());
		SmartDashboard.putNumber(motorName + " VelocityConversionFactor", encoder.getVelocityConversionFactor());

		// Motor Telemetry
		SmartDashboard.putNumber(motorName + " Temperature", motor.getMotorTemperature());
		SmartDashboard.putNumber(motorName + " OutputCurrent", motor.getOutputCurrent());

	}

}
