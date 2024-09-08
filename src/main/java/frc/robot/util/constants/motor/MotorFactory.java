package frc.robot.util.constants.motor;

import frc.robot.util.constants.Device;

public class MotorFactory {
    public static Motor getMotorSpecs(Device hardware) {
        switch (hardware) {
            case NEO:
                return new NEO();
            case NEO_550:
                return new NEO550();
            case VORTEX:
                return new Vortex();
            default:
                throw new IllegalArgumentException("Unknown motor type");
        }
    }
}
