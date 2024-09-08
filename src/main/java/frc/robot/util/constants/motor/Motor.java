package frc.robot.util.constants.motor;

public interface Motor {
    int getCurrentLimit();

    double getMaxVelocity();

    double getFreeRunningCurrent();

    int getStallCurrent();

    double getStallTorque();

    double getPeakOutputPower();

    int getEncoderResolution();
}

class NEO implements Motor {

    @Override
    public int getCurrentLimit() {
        return 40;
    }

    @Override
    public double getMaxVelocity() {
        return 5676.0; // Free speed
    }

    @Override
    public double getFreeRunningCurrent() {
        return 1.8; // Free running current in A
    }

    @Override
    public int getStallCurrent() {
        return 105; // Empirical stall current in A
    }

    @Override
    public double getStallTorque() {
        return 2.6; // Empirical stall torque in Nm
    }

    @Override
    public double getPeakOutputPower() {
        return 406.0; // Empirical peak output power in W
    }

    @Override
    public int getEncoderResolution() {
        return 42; // Encoder resolution in counts per rev
    }
}

class NEO550 implements Motor {

    @Override
    public int getCurrentLimit() {
        return 20;
    }

    @Override
    public double getMaxVelocity() {
        return 11000.0; // Free speed
    }

    @Override
    public double getFreeRunningCurrent() {
        return 1.4; // Free running current in A
    }

    @Override
    public int getStallCurrent() {
        return 100; // Stall current in A
    }

    @Override
    public double getStallTorque() {
        return 0.97; // Stall torque in Nm
    }

    @Override
    public double getPeakOutputPower() {
        return 279.0; // Peak output power in W
    }

    @Override
    public int getEncoderResolution() {
        return 42; // Encoder resolution in counts per rev
    }
}

class Vortex implements Motor {

    @Override
    public int getCurrentLimit() {
        return 40;
    }

    @Override
    public double getMaxVelocity() {
        return 6784.0; // Free speed
    }

    @Override
    public double getFreeRunningCurrent() {
        return 3.6; // Free running current in A
    }

    @Override
    public int getStallCurrent() {
        return 211; // Stall current in A
    }

    @Override
    public double getStallTorque() {
        return 3.6; // Stall torque in Nm
    }

    @Override
    public double getPeakOutputPower() {
        return 640.0; // Peak output power in W
    }

    @Override
    public int getEncoderResolution() {
        return 7168; // Encoder resolution in counts per rev
    }
}
