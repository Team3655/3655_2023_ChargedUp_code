// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.TractorToolbox.TractorParts;

/** Add your docs here. */
public class DoubleSmoother {

	double previousValue;
	double smoothingPercentage;
	double inputSmoothingPercentage;

	public DoubleSmoother() {
		this(.5);
	}

	/**
	 * creates a smoother to be used for motion smoothing
	 * @param smoothingPercentage the smoothing value (0 - 1). this is the percentage of the previous value that will be used.
	 */
	public DoubleSmoother(double smoothingPercentage) {
		this.smoothingPercentage = smoothingPercentage;
		this.inputSmoothingPercentage = 1 - smoothingPercentage;
		previousValue = 0;
	}

	public double smoothInput(double input) {
		double output = (input * inputSmoothingPercentage) + (previousValue * smoothingPercentage);
		previousValue = output;
		return output;
	}

	public double getPercentage() {
		return smoothingPercentage;
	}

}
