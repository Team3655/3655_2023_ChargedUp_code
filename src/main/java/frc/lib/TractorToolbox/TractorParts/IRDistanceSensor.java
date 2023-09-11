package frc.lib.TractorToolbox.TractorParts;

import edu.wpi.first.wpilibj.AnalogInput;

public class IRDistanceSensor {
    private final AnalogInput input;

    public IRDistanceSensor(int port){
        input = new AnalogInput(port);
    }

    public double getDistanceAsVolts() {
        return input.getVoltage();
    }
}