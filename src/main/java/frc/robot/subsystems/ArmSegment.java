package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSegment {

	// region properties

	private CANSparkMax m_rightMotor;
	private CANSparkMax m_leftMotor;

	private SparkMaxPIDController m_PIDController;

	private RelativeEncoder m_rightEncoder;
	private RelativeEncoder m_leftEncoder;

	private double m_targetTheta;
	private double m_realTheta;

	private double m_gearRatio;

	// endregion

	public ArmSegment(int rightPort, int leftPort, double gearRatio) {

		this.m_gearRatio = gearRatio;

		// region def_motors
		// creates left and right arm motors
		m_rightMotor = new CANSparkMax(leftPort, MotorType.kBrushless);
		m_leftMotor = new CANSparkMax(rightPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		m_rightMotor.restoreFactoryDefaults();
		m_leftMotor.restoreFactoryDefaults();

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		m_PIDController = m_rightMotor.getPIDController();

		// Encoder object created to display position values
		m_rightEncoder = m_rightMotor.getEncoder();
		m_rightEncoder = m_rightMotor.getEncoder();

		// motors to invert
		m_leftMotor.setInverted(true);

		// set current limits
		m_rightMotor.setSmartCurrentLimit(30);
		m_leftMotor.setSmartCurrentLimit(30);

		// sets motor defaults to break
		m_rightMotor.setIdleMode(IdleMode.kBrake);
		m_leftMotor.setIdleMode(IdleMode.kBrake);

		// set PID coefficients
		m_PIDController.setP(0);
		m_PIDController.setI(0);
		m_PIDController.setD(0);
		m_PIDController.setOutputRange(-1, 1);

		// sets left motor to follow right
		m_leftMotor.follow(m_rightMotor);
		// endregion

	}

	// region: setters

	/** Sets the pid referance point to the arc length of the target angle */
	public void setReference() {
		m_PIDController.setReference(
				getThetaToTicks(m_targetTheta),
				CANSparkMax.ControlType.kPosition);
	}

	/**
	 * Sets the target angle of the arm
	 * 
	 * @param theta the target angle to be set
	 */
	public void setTagetTheta(double theta) {
		m_targetTheta = Math.toRadians(theta);
		
	}

	/**
	 * sets the PID values for the arm segment
	 * 
	 * @param P Just look up a PID loop
	 * @param I Seriously hurry up!
	 * @param D Don't make me wait on behalf of your ignorance
	 */
	public void setPID(double P, double I, double D) {
		m_PIDController.setP(P);
		m_PIDController.setI(I);
		m_PIDController.setD(D);
	}

	// endregion

	// region: getters

	/**
	 * Used for getting the number of ticks required to turn an angle
	 * 
	 * @param theta      the angke you want to turn (in radians)
	 * @param totalTicks the number of ticks required to make one revolution
	 * @return the number of motor ticks required to turn theta
	 */
	public int getThetaToTicks(double theta) {
		return (int) (theta * (double) m_gearRatio);
	}

	// endregion
}
