package frc.robot.util.hardware;

import frc.robot.util.constants.Device;

public class CANDevice {

    private String subsystem;
    private String deviceName;
    private int canId;
    private Device device;

    public CANDevice(String subsystem, String deviceName, int canId, Device device) {
        this.subsystem = subsystem;
        this.deviceName = deviceName;
        this.canId = canId;
        this.device = device;
    }

    public String getSubsystem() {
        return subsystem;
    }

    public String getDeviceName() {
        return deviceName;
    }

    public int getCanId() {
        return canId;
    }

    public Device getDevice() {
        return device;
    }

    @Override
    public String toString() {
        return "CANDevice{" +
                "subsystem='" + subsystem + '\'' +
                ", deviceName='" + deviceName + '\'' +
                ", canId=" + canId +
                '}';
    }
}
