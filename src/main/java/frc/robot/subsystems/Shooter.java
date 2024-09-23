package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.NoteTracker;
import frc.robot.util.hardware.SparkFlex;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {

    // Controllers
    private final SparkFlex leftWheels = new SparkFlex(CANBus.SHOOTER_LEFT);
    private final SparkFlex rightWheels = new SparkFlex(CANBus.SHOOTER_RIGHT);

    // Feedforward controller (coefficients will be updated after SysId)
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    // State
    private final NoteTracker noteTracker;
    private int velocitySetpoint = MotorSetPoint.STOP;

    // Mutable holders for SysId routine
    private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> positionMeasure = MutableMeasure.mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> velocityMeasure = MutableMeasure.mutable(RadiansPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> {
                leftWheels.setVoltage(volts.in(Volts));
                rightWheels.setVoltage(volts.in(Volts));
            },
            log -> {
                log.motor("Shooter Flywheel")
                   .voltage(appliedVoltage.mut_replace(getCurrentVoltage(), Volts))
                   .angularPosition(positionMeasure.mut_replace(getShooterPositionRad(), Radians))
                   .angularVelocity(velocityMeasure.mut_replace(getShooterVelocityRad(), RadiansPerSecond));
            },
            this
        )
    );

    public Shooter(NoteTracker noteTracker) {
        this.noteTracker = noteTracker;

        // Set motor controller configurations
        leftWheels.setIdleMode(IdleMode.kCoast);
        leftWheels.setInverted(false);
        leftWheels.pidParameters.SetPID(0.0001, 0.000001, 0);
        leftWheels.burnFlash();

        rightWheels.setIdleMode(IdleMode.kCoast);
        rightWheels.setInverted(true);
        rightWheels.pidParameters.SetPID(0.0001, 0.000001, 0);
        rightWheels.burnFlash();
    }

    @Override
    public void periodic() {
        leftWheels.periodic();
        rightWheels.periodic();
    }

    public void primeShooter() {
        // Set the velocity setpoint
        velocitySetpoint = MotorSetPoint.SHOOTER_VELOCITY;

        // Calculate feedforward output in volts
        double ffVolts = feedforward.calculate(MotorSetPoint.SHOOTER_VELOCITY / 60.0); // Convert RPM to RPS

        // Set the target RPM using PID control with feedforward
        leftWheels.pidController.setReference(MotorSetPoint.SHOOTER_VELOCITY, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
        rightWheels.pidController.setReference(MotorSetPoint.SHOOTER_VELOCITY, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
    }

    public void stopShooter() {
        velocitySetpoint = MotorSetPoint.STOP;
        leftWheels.stopMotor();
        rightWheels.stopMotor();
    }

    public boolean isShooterReady() {
        // Get the current velocities of both wheels
        double leftWheelVelocity = leftWheels.encoder.getVelocity();
        double rightWheelVelocity = rightWheels.encoder.getVelocity();

        // Check if both wheels are within the tolerance of the setpoint
        boolean isVelocityOnTarget = Math.abs(leftWheelVelocity - velocitySetpoint) <= MotorSetPoint.SHOOTER_VELOCITY_TOLERANCE &&
            Math.abs(rightWheelVelocity - velocitySetpoint) <= MotorSetPoint.SHOOTER_VELOCITY_TOLERANCE;

        // The shooter is ready if the velocity is on target and there's a note ready
        return isVelocityOnTarget && noteTracker.hasNote();
    }

    private double getCurrentVoltage() {
        return RobotController.getBatteryVoltage();
    }

    private double getShooterPositionRad() {
        // Assuming left and right wheels are synchronized
        return leftWheels.encoder.getPosition() * 2.0 * Math.PI;
    }

    private double getShooterVelocityRad() {
        // Assuming left and right wheels are synchronized
        return leftWheels.encoder.getVelocity() * 2.0 * Math.PI / 60.0; // Convert RPM to rad/s
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
