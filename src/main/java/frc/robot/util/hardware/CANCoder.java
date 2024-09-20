package frc.robot.util.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.util.constants.Device;

/**
 * A wrapper class for the CANcoder that adds an inverted property and
 * stores CANDevice properties for easy access.
 */
public class CANCoder extends CANcoder {
    private boolean inverted;
    private CANDevice canDevice;

    /**
     * Constructs a new CANCoder object with specified CANDevice and default inversion (not inverted).
     *
     * @param canDevice CANDevice object containing device information.
     */
    public CANCoder(CANDevice canDevice) {
        this(canDevice, false);
    }

    /**
     * Constructs a new CANCoder object with specified CANDevice and inversion state.
     *
     * @param canDevice CANDevice object containing device information.
     * @param inverted  Initial inversion state.
     */
    public CANCoder(CANDevice canDevice, boolean inverted) {
        super(canDevice.getCanId());
        this.canDevice = canDevice;
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
     * Returns the CANDevice associated with this CANCoder.
     *
     * @return the CANDevice object.
     */
    public CANDevice getCANDevice() {
        return canDevice;
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

    // Accessor methods for CANDevice properties

    /**
     * Returns the subsystem of the CANDevice.
     *
     * @return Subsystem name.
     */
    public String getSubsystem() {
        return canDevice.getSubsystem();
    }

    /**
     * Returns the device name of the CANDevice.
     *
     * @return Device name.
     */
    public String getDeviceName() {
        return canDevice.getDeviceName();
    }

    /**
     * Returns the CAN ID of the CANDevice.
     *
     * @return CAN ID.
     */
    public int getCanId() {
        return canDevice.getCanId();
    }

    /**
     * Returns the Device enum of the CANDevice.
     *
     * @return Device enum.
     */
    public Device getDevice() {
        return canDevice.getDevice();
    }
}
