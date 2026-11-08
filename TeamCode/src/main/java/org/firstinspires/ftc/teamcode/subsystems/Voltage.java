package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * An extended Voltage wrapper class that prevents multiple reads in the same loop.
 */
public class Voltage {
    private double lastReading = 12;
    private final VoltageSensor voltageSensor;

    // Constructor
    public Voltage(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    // Getter
    public VoltageSensor getSensor(){
        return voltageSensor;
    }

    // Get and set voltage value
    public double updateVoltage() {
        lastReading = voltageSensor.getVoltage();
        return lastReading;
    }

    // Get voltage value
    public double getVoltage() {
        return lastReading;
    }
}
