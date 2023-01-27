package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
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
		TUCKED,
		LOW_SCORE,
		MID_SCORE,
		HIGH_SCORE,
		LOW_INTAKE,
		MID_INTAKE,
		HIGH_INTAKE,
		DRIVER_CONTROL
	}

	/** controls the side of the robot the arm is on */
	private boolean isFront;

	/** the variable setting the height of the arm */
	ArmPoses armState;
	ArmPoses prevArmState;

	/** the target angle for the major arm in Degrees */
	double majorArmTargetTheta;
	/** the target angle for the minor arm in Degrees */
	double minorArmTargetTheta;

	// endregion

	public ArmSubsystem() {

		// the default state of the arms
		isFront = true;
		armState = ArmPoses.TUCKED;
		prevArmState = armState;

		// region def motors
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

		// sets motor defaults to break
		rightMajorMotor.setIdleMode(IdleMode.kBrake);
		leftMajorMotor.setIdleMode(IdleMode.kBrake);
		rightMinorMotor.setIdleMode(IdleMode.kBrake);
		leftMinorMotor.setIdleMode(IdleMode.kBrake);

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
		// endregion
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// skips math if state has not changed
		if (prevArmState != armState) {
			switch (armState) {
				// When the arms are tucked in the center of the robot (this is the only legal
				// starting position)
				case TUCKED:
					majorArmTargetTheta = 0;
					minorArmTargetTheta = 0;
					break;

				// Used for scoring in the lowest "hybrid" node
				case LOW_SCORE:
					majorArmTargetTheta = 45;
					minorArmTargetTheta = 90;
					break;

				// Used for scoring in the middle node
				case MID_SCORE:
					majorArmTargetTheta = 75;
					minorArmTargetTheta = 90;
					break;

				// Used for scoring in the highest node
				case HIGH_SCORE:
					majorArmTargetTheta = 90;
					minorArmTargetTheta = 90;
					break;

				// Used for intaking off of the floor
				case LOW_INTAKE:
					majorArmTargetTheta = 30;
					minorArmTargetTheta = 100;
					break;

				// Used for intaking from the human player chute
				case MID_INTAKE:
					majorArmTargetTheta = 30;
					minorArmTargetTheta = 100;
					break;

				// Used for intaking from the sliding human player station
				case HIGH_INTAKE:
					majorArmTargetTheta = 80;
					minorArmTargetTheta = 80;
					break;

				// goes to the pair of angles defined my the TSB driver
				case DRIVER_CONTROL:
					// Constrains the major arm to stay between 0 and 90 degrees
					if (majorArmTargetTheta < 0) {
						majorArmTargetTheta = 0;
					} else if (majorArmTargetTheta > 90) {
						majorArmTargetTheta = 90;
					}

					// Constrains the minor arm to stay between 0 and 90 degrees
					if (minorArmTargetTheta < -90) {
						minorArmTargetTheta = 0;
					} else if (minorArmTargetTheta > 90) {
						minorArmTargetTheta = 90;
					}
					break;
			}

			// Offset the minor arm based on the angle of the major arm (this makes the
			// minor arm reletive to the robot)
			minorArmTargetTheta += majorArmTargetTheta;

		}

		// Swaps the sign of the target angle if the dominant side of the robot is back
		if (!isFront) {
			majorArmTargetTheta = Math.copySign(majorArmTargetTheta, -1);
			minorArmTargetTheta = Math.copySign(majorArmTargetTheta, -1);
		}

		// Address the major motors
		majorPIDController.setReference(
				getThetaToTicks(Math.toRadians(majorArmTargetTheta * ArmConstants.kMajorArmDir),
						ArmConstants.kMajorArmTicks),
				CANSparkMax.ControlType.kPosition);

		// Address the minor motors
		minorPIDController.setReference(
				getThetaToTicks(Math.toRadians(minorArmTargetTheta * ArmConstants.kMinorArmDir),
						ArmConstants.kMinorArmTicks),
				CANSparkMax.ControlType.kPosition);

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

	/** ruturns true if the target dominant side of the robot is front */
	public boolean getIsFront() {
		return isFront;
	}

	/**
	 * This method is used to get the number of revolutions the arm motor has made
	 * 
	 * @param isMajor setting this to false will return the count of the minor arm
	 * @return the number of revolutions as a double
	 */
	public double getRightEncoderTicks(boolean isMajor) {
		if (isMajor) {
			return rightMajorEncoder.getPosition();
		}
		return rightMinorEncoder.getPosition();
	}

	/**
	 * Used for getting the number of ticks required to turn an angle
	 * 
	 * @param theta the angke you want to turn (in radians)
	 * @param ticks the number of ticks required to make one revolution
	 * @return the number of motor ticks required to turn theta
	 */
	public int getThetaToTicks(double theta, int ticks) {
		return (int) (theta * (double) ticks);
	}

	// endregion

}
