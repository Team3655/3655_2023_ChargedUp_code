package frc.robot.Objects;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.*;

public class Limelight {

	private boolean useVision;
	private boolean useTape;
	private boolean hasValidTarget;

	private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
	private double tx = limelight.getEntry("tx").getDouble(0);
	private double ty = limelight.getEntry("ty").getDouble(0);
	private double ta = limelight.getEntry("ta").getDouble(0);
	private double tv = limelight.getEntry("tv").getDouble(0);

	public Limelight() {
		useVision = true;
		useTape = true;
		hasValidTarget = false;
	}

	public void updateLimelight() {	

		// skips updating vision if the limelight is set to DriverCam
		if (useVision) {

			// changes what pipeline the light gets from
			if (useTape) {
				updateReflectiveTape();
			} else {
				updateAprilTags();
			}

		} else {
			disable();
		}

	}

	// region getters
	public void updateReflectiveTape() {

		tx = limelight.getEntry("tx").getDouble(0);
		ty = limelight.getEntry("ty").getDouble(0);
		ta = limelight.getEntry("ta").getDouble(0);

		hasValidTarget = (limelight.getEntry("tv").getDouble(0) < 1) ? false : true;

	}

	public void updateAprilTags() {

	}

	/** takes a snapshot for later use */
	public void takeSnapshot() {
		limelight.getEntry("snapshot").setNumber(1);
	}

	// endregion


	// region setters

	/**
	 * sets cam mode to Driver Camera (Increases exposure, disables vision
	 * processing)
	 */
	public void disable() {
		useVision = false;
		limelight.getEntry("camMode").setNumber(1);
	}

	/** sets cam mode to Vision processor */
	public void enable() {
		useVision = true;
		limelight.getEntry("camMode").setNumber(0);
	}

	/**
	 * Sets the limelights current pipeline
	 * 
	 * @param pipeline the ID of the pipeline to be set
	 */
	public void setPipeline(int pipeline) {
		limelight.getEntry("pipeline").setNumber(pipeline);
	}

	// endregion

}
