// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * A wrapper class for the CANcoder that adds an inverted property and
 * a method to easily change the sensor direction.
 */
public class CANCoder extends CANcoder {
    private boolean inverted;

    /**
     * Constructs a new CANCoder object with default inversion (not inverted).
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     */
    public CANCoder(int deviceId) {
        super(deviceId);
        this.inverted = false;
        applyInversion();
    }

    /**
     * Constructs a new CANCoder object with specified inversion.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param inverted Initial inversion state.
     */
    public CANCoder(int deviceId, boolean inverted) {
        super(deviceId);
        this.inverted = inverted;
        applyInversion();
    }

    /**
     * Constructs a new CANCoder object with default inversion (not inverted)
     * and specified CAN bus.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param canbus   Name of the CAN bus this device is on.
     */
    public CANCoder(int deviceId, String canbus) {
        super(deviceId, canbus);
        this.inverted = false;
        applyInversion();
    }

    /**
     * Constructs a new CANCoder object with specified inversion and CAN bus.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param canbus   Name of the CAN bus this device is on.
     * @param inverted Initial inversion state.
     */
    public CANCoder(int deviceId, String canbus, boolean inverted) {
        super(deviceId, canbus);
        this.inverted = inverted;
        applyInversion();
    }

    /**
     * Returns the current inversion state.
     *
     * @return true if inverted, false otherwise.
     */
    public boolean isInverted() {
        return inverted;
    }

    /**
     * Sets the inversion state and updates the sensor direction accordingly.
     *
     * @param inverted New inversion state.
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        applyInversion();
    }

    /**
     * Applies the inversion setting to the CANcoder configuration.
     */
    private void applyInversion() {
        CANcoderConfigurator canCoderConfigurator = this.getConfigurator();
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Refresh the current configuration
        canCoderConfigurator.refresh(config);

        // Set the sensor direction based on the inversion state
        config.MagnetSensor.SensorDirection = inverted
                ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        // Apply the updated configuration
        canCoderConfigurator.apply(config);
    }
}
