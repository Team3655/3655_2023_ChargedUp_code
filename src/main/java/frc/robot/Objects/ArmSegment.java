package frc.robot.Objects;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSegment {

	// region properties

	/** Motor controllers for the segment */
	private CANSparkMax m_rightMotor, m_leftMotor;

	/** PID controllers for the segment */
	public SparkMaxPIDController m_rightPIDController, m_leftPIDController;

	/** Encoders for the segment */
	private RelativeEncoder m_rightEncoder, m_leftEncoder;

	/** Real and target Angles for the arms */
	private double m_targetTheta, m_realTheta;

	/** Defined by the number of motor rotations per revolution of the arm (360°) */
	private double m_gearRatio, m_gearRatioRadius;

	/** the angle constraints on the arm */
	private double m_minTheta, m_maxTheta;

	/** Controls the direction of the arm */
	private int m_targetSign;

	// endregion

	public ArmSegment(int rightPort, int leftPort, double gearRatio, Boolean invertLeft) {

		m_targetSign = 1;
		this.m_gearRatio = gearRatio;
		this.m_gearRatioRadius = gearRatio / (2 * Math.PI);

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

		// sets motor defaults to break
		m_rightMotor.setIdleMode(IdleMode.kBrake);
		m_leftMotor.setIdleMode(IdleMode.kBrake);

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		m_rightPIDController = m_rightMotor.getPIDController();
		m_leftPIDController = m_leftMotor.getPIDController();

		// Encoder object created to display position values
		m_rightEncoder = m_rightMotor.getEncoder();
		m_leftEncoder = m_leftMotor.getEncoder();

		m_rightEncoder.setPositionConversionFactor((2 * Math.PI) / m_gearRatio);
		m_leftEncoder.setPositionConversionFactor((2 * Math.PI) / m_gearRatio);

		// set PID coefficients
		m_rightPIDController.setP(0);
		m_rightPIDController.setI(0);
		m_rightPIDController.setD(0);
		m_rightPIDController.setOutputRange(-0.5, 0.5);

		m_leftPIDController.setP(0);
		m_leftPIDController.setI(0);
		m_leftPIDController.setD(0);
		m_leftPIDController.setOutputRange(-0.5, 0.5);

		// Sets the left motor to be inverted if it needs to be
		m_leftMotor.setInverted(invertLeft);

		// endregion
	}

	// region: setters

	/** Sets the pid referance point to the target theta of the segment */
	public void setReference() {
		m_rightPIDController.setReference(m_targetTheta * m_targetSign, CANSparkMax.ControlType.kPosition);
		m_leftPIDController.setReference(m_targetTheta * m_targetSign, CANSparkMax.ControlType.kPosition);
	}

	/**
	 * Sets the sign that controls the dominant side of the robot
	 * 
	 * @param sign the Sign to set
	 */
	public void setSign(int sign) {
		m_targetSign = sign;
	}

	/** sets the min and max target angle of */
	public void setConstraints(int min, int max) {
		m_maxTheta = max;
		m_minTheta = min;
	}

	/**
	 * Forces a given angle to be between thw min and max constraints
	 * 
	 * @param theta the angle to be constrained
	 * @return the limited value of theta
	 */
	public double constrain(double theta) {

		if (theta >= m_maxTheta) {
			return m_maxTheta;

		} else if (theta <= m_minTheta) {
			return m_minTheta;
		}

		return theta;
	}

	/**
	 * Sets the target angle of the arm IN RADIANS!
	 * 
	 * @param theta the target angle to be set
	 */
	public void setTargetTheta(double theta) {
		theta = constrain(theta);
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
		m_rightPIDController.setP(P);
		m_rightPIDController.setI(I);
		m_rightPIDController.setD(D);
		m_leftPIDController.setP(P);
		m_leftPIDController.setI(I);
		m_leftPIDController.setD(D);
	}

	/**
	 * Used to set the maximum power draw of the arm
	 * 
	 * @param limit the maximum allowed power draw
	 */
	public void setSmartCurrentLimit(int limit) {
		m_rightMotor.setSmartCurrentLimit(limit);
		m_leftMotor.setSmartCurrentLimit(limit);
	}

	// endregion

	// region: getters

	/**
	 * @return the m_targetTheta
	 */
	public double getTargetTheta() {
		return m_targetTheta;
	}

	/**
	 * (Depreciated) Used for getting the number of ticks required to turn an angle
	 * 
	 * @param theta      the angke you want to turn (in radians)
	 * @param totalTicks the number of ticks required to make one revolution
	 * @return the number of motor ticks required to turn theta
	 */
	public double getThetaToTicks() {
		return m_targetTheta * m_gearRatioRadius;
	}

	/** Returns the actual angle of the real arm (not the same as the target) */
	public double getRealTheta() {
		return Math.toDegrees((m_rightEncoder.getPosition() + m_leftEncoder.getPosition()) / 2);
	}

	/**
	 * Checks of the arm is at its target position
	 * 
	 * @param deadBand the number of degrees of error that is acceptable
	 * @return True if the Real angle is within the deadband of the target
	 */
	public boolean getAtTarget(double deadBand) {
		// get absolute value of the difference
		double error = Math.abs(getRealTheta() - m_targetTheta);
		// convert deadband to radians
		deadBand = Math.toRadians(deadBand);

		if (error < deadBand) {
			return true;
		}
		return false;
	}

	// endregion

}
