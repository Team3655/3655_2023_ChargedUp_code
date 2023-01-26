package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	// the motor objects for the motors controling the arms
	private CANSparkMax rightMajorMotor;
	private CANSparkMax leftMajorMotor;
	private CANSparkMax rightMinorMotor;
	private CANSparkMax leftMinorMotor;

	// the pid controlers for the major and minor arms
	private SparkMaxPIDController majorPIDController;
	private SparkMaxPIDController minorPIDController;

	private RelativeEncoder rightMajorEncoder;
	private RelativeEncoder rightMinorEncoder;

	/** used for controling the height of the arm */
	private enum ArmPoses {
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_INTAKE,
		MID_INTAKE,
		HIGH_INTAKE
	}

	/** controls when the arm is tucked */
	private boolean isTucked;

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	/** the variable setting the height of the arm */
	ArmPoses armState;

	/** sets the target angle for the major arm */
	float majorArmTargetTheta;
	/** sets the target angle for the minor arm */
	float minorArmTargetTheta;

	// endregion

	public ArmSubsystem() {

		// the default state of the arms
		isTucked = true;
		isFront = true;
		armState = ArmPoses.LOW_INTAKE;

		// creates the arms motors
		rightMajorMotor = new CANSparkMax(ArmConstants.kRightMajorArmPort, MotorType.kBrushless);
		leftMajorMotor = new CANSparkMax(ArmConstants.kLeftMajorArmPort, MotorType.kBrushless);
		rightMinorMotor = new CANSparkMax(ArmConstants.kRightMinorArmPort, MotorType.kBrushless);
		leftMinorMotor = new CANSparkMax(ArmConstants.kLeftMinorArmPort, MotorType.kBrushless);

		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		rightMajorMotor.restoreFactoryDefaults();
		leftMajorMotor.restoreFactoryDefaults();
		rightMinorMotor.restoreFactoryDefaults();
		leftMinorMotor.restoreFactoryDefaults();

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		majorPIDController = rightMajorMotor.getPIDController();
		minorPIDController = rightMajorMotor.getPIDController();

		// Encoder object created to display position values
		rightMajorEncoder = rightMajorMotor.getEncoder();
		rightMinorEncoder = rightMinorMotor.getEncoder();

		// motors to invert
		leftMajorMotor.setInverted(true);

		// set current limits
		rightMajorMotor.setSmartCurrentLimit(30);
		leftMajorMotor.setSmartCurrentLimit(30);
		rightMinorMotor.setSmartCurrentLimit(30);
		leftMinorMotor.setSmartCurrentLimit(30);

		// set PID coefficients
		majorPIDController.setP(ArmConstants.kMajorArmP);
		majorPIDController.setI(ArmConstants.kMajorArmI);
		majorPIDController.setD(ArmConstants.kMajorArmD);
		majorPIDController.setIZone(0);
		majorPIDController.setFF(0);
		majorPIDController.setOutputRange(-1, 1);

		// set PID coefficients
		minorPIDController.setP(ArmConstants.kMinorArmP);
		minorPIDController.setI(ArmConstants.kMinorArmI);
		minorPIDController.setD(ArmConstants.kMinorArmD);
		minorPIDController.setIZone(0);
		minorPIDController.setFF(0);
		minorPIDController.setOutputRange(-1, 1);

		leftMajorMotor.follow(rightMajorMotor);
		leftMinorMotor.follow(rightMinorMotor);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	// region setters

	/**
	 * Sets the height of the arm
	 * 
	 * @param pos can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE, MID_INTAKE,
	 *            HIGH_INTAKE)
	 */
	public void setArmState(ArmPoses pos) {
		armState = pos;
	}

	/**
	 * Sets the dominant side of te robot
	 * 
	 * @param side if true the front will be dominant
	 */
	public void setFront(boolean side) {
		isFront = side;
	}

	/**
	 * Toggles the dominant side of the robot
	 */
	public void setToggleSide() {
		isFront = !isFront;
	}

	/**
	 * sets the robot to be tucked
	 * 
	 * @param tuck if true the robot will tuck itself
	 */
	public void setTucked(boolean tuck) {
		isTucked = tuck;
	}

	// endregion

	// region getters

	/**
	 * Used to get the target height of the arm as an enum (arm will not be at this
	 * height if
	 * tucked)
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public ArmPoses getArmState() {
		return armState;
	}

	/** ruturns true if the robots target state is tucked */
	public boolean getIsTucked() {
		return isTucked;
	}

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	/**
	 * Used for getting the number of ticks required to turn an angle
	 * 
	 * @param theta the angke you want to turn (in radians)
	 * @param ticks the number of ticks required to make one revolution
	 * @return the number of motor ticks required to turn theta
	 */
	public int getThetaToTicks(float theta, int ticks) {
		return (int) (theta * (float) ticks);
	}

	// endregion

}
