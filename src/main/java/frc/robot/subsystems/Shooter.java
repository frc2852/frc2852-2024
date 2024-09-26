package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.NoteTracker;
import frc.robot.util.hardware.SparkFlex;

public class Shooter extends SubsystemBase {

    // Controllers
    private final SparkFlex leftWheels = new SparkFlex(CANBus.SHOOTER_LEFT);
    private final SparkFlex rightWheels = new SparkFlex(CANBus.SHOOTER_RIGHT);

    // Feedforward controller (coefficients will be updated after SysId)
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.24697, 0.10046, 0.0084684);

    // State
    private final NoteTracker noteTracker;
    private int velocityLeftSetpoint = MotorSetPoint.STOP;
    private int velocityRightSetpoint = MotorSetPoint.STOP;

    // Mutable holders for SysId routine
    private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    private final MutableMeasure<Angle> positionMeasure = MutableMeasure.mutable(Units.Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocityMeasure = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    volts -> {
                        double voltageValue = volts.in(Units.Volts);
                        appliedVoltage.mut_replace(voltageValue, Units.Volts);
                        leftWheels.setVoltage(voltageValue);
                    },
                    log -> {
                        log.motor("ShooterFlywheel")
                                .voltage(appliedVoltage)
                                .angularPosition(positionMeasure.mut_replace(leftWheels.encoder.getPosition(), Units.Rotations))
                                .angularVelocity(velocityMeasure.mut_replace(leftWheels.encoder.getVelocity() / 60.0, Units.RotationsPerSecond));
                    },
                    this));

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
    }

    public void primeShooter() {
        // Set the velocity setpoint
        velocityLeftSetpoint = MotorSetPoint.SHOOTER_LEFT_VELOCITY;
        velocityRightSetpoint = MotorSetPoint.SHOOTER_RIGHT_VELOCITY;

        // Calculate feedforward output in volts
        double ffVoltsLeft = feedforward.calculate(velocityLeftSetpoint / 60.0); // Convert RPM to RPS
        double ffVoltsRight = feedforward.calculate(velocityRightSetpoint / 60.0); // Convert RPM to RPS

        leftWheels.pidController.setReference(velocityLeftSetpoint, ControlType.kVelocity, 0, ffVoltsLeft, ArbFFUnits.kVoltage);
        rightWheels.pidController.setReference(velocityRightSetpoint, ControlType.kVelocity, 0, ffVoltsRight, ArbFFUnits.kVoltage);
    }

    public void stopShooter() {
        velocityLeftSetpoint = MotorSetPoint.STOP;
        velocityRightSetpoint = MotorSetPoint.STOP;

        leftWheels.stopMotor();
        rightWheels.stopMotor();
    }

    public boolean isShooterReady() {
        // Get the current velocities of both wheels
        double leftWheelVelocity = leftWheels.encoder.getVelocity();
        double rightWheelVelocity = rightWheels.encoder.getVelocity();

        // Check if both wheels are within the tolerance of the setpoint
        boolean isVelocityOnTarget = Math.abs(leftWheelVelocity - velocityLeftSetpoint) <= MotorSetPoint.SHOOTER_VELOCITY_TOLERANCE &&
                Math.abs(rightWheelVelocity - velocityRightSetpoint) <= MotorSetPoint.SHOOTER_VELOCITY_TOLERANCE;

        // The shooter is ready if the velocity is on target and there's a note ready
        return isVelocityOnTarget && noteTracker.hasNote();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
