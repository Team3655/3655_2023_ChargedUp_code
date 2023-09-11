package frc.robot.Mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import frc.robot.TractorToolbox.TractorParts.PIDGains;

public class ArmSegment {

	// region properties

	/** Motor controllers for the segment */
	private CANSparkMax rightMotor, leftMotor;

	/** PID controllers for the segment */
	public SparkMaxPIDController rightPIDController;

	/** Encoders for the segment */
	private RelativeEncoder rightEncoder, leftEncoder;

	/** Real and target Angles for the arms */
	private double targetTheta;

	/** the angle constraints on the arm */
	private double minTheta, maxTheta;

	/** Controls the direction of the arm */
	private int targetSign;

	private boolean isRunning;

	// endregion

	public ArmSegment(int rightPort, int leftPort, double gearRatio, Boolean invertLeader) {

		targetSign = 1;
		isRunning = true;

		// region def_motors
		// creates left and right arm motors
		rightMotor = new CANSparkMax(rightPort, MotorType.kBrushless);
		leftMotor = new CANSparkMax(leftPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		rightMotor.restoreFactoryDefaults();
		leftMotor.restoreFactoryDefaults();

		// sets motor defaults to break
		rightMotor.setIdleMode(IdleMode.kCoast);
		leftMotor.setIdleMode(IdleMode.kCoast);

		// Encoder object created to display position values
		rightEncoder = rightMotor.getEncoder();
		leftEncoder = leftMotor.getEncoder();

		// Tells the motors to automatically convert degrees to rotations
		rightEncoder.setPositionConversionFactor((2 * Math.PI) / gearRatio);
		leftEncoder.setPositionConversionFactor((2 * Math.PI) / gearRatio);
		rightEncoder.setVelocityConversionFactor((2 * Math.PI) / gearRatio);
		leftEncoder.setVelocityConversionFactor((2 * Math.PI) / gearRatio);

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		rightPIDController = rightMotor.getPIDController();

		// tells the pid controller on the arms to use trapezoidal constraints 
		rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

		rightPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

		rightMotor.setInverted(invertLeader);
		leftMotor.follow(rightMotor, true);

		// rightMotor.set(0);
		// leftMotor.set(0);

		// endregion
	}

	// region: setters

	/** Sets the pid referance point to the target theta of the segment */
	public void setReference() {
		rightPIDController.setReference(targetTheta * targetSign, CANSparkMax.ControlType.kSmartMotion);
	}

	/**
	 * Sets the sign that controls the dominant side of the robot
	 * 
	 * @param sign the Sign to set
	 */
	public void setSign(int sign) {
		targetSign = sign;
	}

	/** sets the angle constraint and the speed constraint */
	public void setConstraints(double theta) {
		maxTheta = theta;
		minTheta = -theta;
	}

	/**
	 * sets the maximum output of the pid controller
	 * 
	 * @param maxOutput the max value that can be sent to the motor (0 - 1)
	 */
	public void setMaxOutput(double maxOutput) {
		rightPIDController.setOutputRange(-maxOutput, maxOutput);
	}

	/**
	 * Used to set the maximum velocity and acceleration of the arm
	 * 
	 * @param maxVel the max velocity of the arm in radians per minute
	 * @param maxAccel the max velocity of the arm in radians per minute
 	 */
	public void setTrapazoidalConstraints(double maxVel, double maxAccel) {
		rightPIDController.setSmartMotionMaxVelocity(maxVel, 0);
		rightPIDController.setSmartMotionMinOutputVelocity(0, 0);
		rightPIDController.setSmartMotionMaxAccel(maxAccel, 0);
	}

	/**
	 * Forces a given angle to be between thw min and max constraints
	 * 
	 * @param theta the angle to be constrained
	 * @return the limited value of theta
	 */
	public double constrain(double theta) {

		if (theta >= maxTheta) {
			return maxTheta;

		} else if (theta <= minTheta) {
			return minTheta;
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
		targetTheta = Math.toRadians(theta);
	}

	/**
	 * Sets the pis gains of the arms
	 * 
	 * @param inputGains the gains to be set
	 */
	public void setPID(PIDGains inputGains) {
		rightPIDController.setP(inputGains.kP);
		rightPIDController.setI(inputGains.kI);
		rightPIDController.setD(inputGains.kD);
	}

	/**
	 * Used to set the maximum power draw of the arm
	 * 
	 * @param limit the maximum allowed power draw
	 */
	public void setSmartCurrentLimit(int limit) {
		rightMotor.setSmartCurrentLimit(limit);
		leftMotor.setSmartCurrentLimit(limit);
		rightMotor.setSecondaryCurrentLimit(limit + 3);
		leftMotor.setSecondaryCurrentLimit(limit + 3);
	}

	/** used to disable the motors for rezeroing */
	public void toggleMotors() {
		isRunning = !isRunning;

		if (isRunning) {
			rightMotor.stopMotor();
			leftMotor.stopMotor();
		} else {
			setReference();
		}
	}

	/** resets the zeros of the arms to their current positions */
	public void resetZeros() {
		rightEncoder.setPosition(0);
		leftEncoder.setPosition(0);
	}

	// endregion

	// region: getters

	/**
	 * @return the targetTheta
	 */
	public double getTargetTheta() {
		return targetTheta;
	}

	/** Returns the actual angle of the real arm (not the same as the target) */
	public double getRealTheta() {
		return Math.toDegrees(rightEncoder.getPosition());
	}

	public double getLeftRealTheta() {
		return Math.toDegrees(leftEncoder.getPosition());
	}

	public double getRightRealTheta() {
		return Math.toDegrees(rightEncoder.getPosition());
	}

	/**
	 * Checks of the arm is at its target position
	 * 
	 * @param deadBand the number of degrees of error that is acceptable
	 * @return True if the Real angle is within the deadband of the target
	 */
	public boolean getAtTarget(double deadBand) {
		// get absolute value of the difference
		double error = Math.abs(Math.abs(getRealTheta()) - Math.toDegrees(Math.abs(targetTheta)));

		if (error < deadBand) {
			return true;
		}
		return false;
	}

	/**
	 * @return the total power draw of the arm in amps
	 */
	public double getPowerDraw() {
		return rightMotor.getOutputCurrent() + leftMotor.getOutputCurrent();
	}

	/**
	 * @return the Output current
	 */
	public double getLeftMotorOutput() {
		return leftMotor.getOutputCurrent();
	}

	/**
	 * @return the Output current
	 */
	public double getRightMotorOutput() {
		return rightMotor.getOutputCurrent();
	}

	// endregion

}
