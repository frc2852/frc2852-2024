package frc.robot.util.hardware;

public class CANDevice {

    private String subsystem;
    private String deviceName;
    private int canId;

    public CANDevice(String subsystem, String deviceName, int canId) {
        this.subsystem = subsystem;
        this.deviceName = deviceName;
        this.canId = canId;
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

    @Override
    public String toString() {
        return "CANDevice{" +
                "subsystem='" + subsystem + '\'' +
                ", deviceName='" + deviceName + '\'' +
                ", canId=" + canId +
                '}';
    }
}