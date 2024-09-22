package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.ConfigurationProperties;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.NoteTracker;
import frc.robot.util.hardware.SparkFlex;

public class Shooter extends SubsystemBase {

  // Controllers
  private final SparkFlex leftWheels = new SparkFlex(CANBus.SHOOTER_LEFT);
  private final SparkFlex rightWheels = new SparkFlex(CANBus.SHOOTER_RIGHT);

  // Feedforward controller (coefficients will be updated after SysId)
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  // State
  private final NoteTracker noteTracker;
  private int velocitySetpoint = MotorSetPoint.STOP;

  // Variables for SysId
  private double sysIdVoltageCommand = 0.0;

  public Shooter(NoteTracker noteTracker) {
    this.noteTracker = noteTracker;

    // Set motor controller configurations
    leftWheels.setIdleMode(IdleMode.kCoast);
    leftWheels.setInverted(true);

    rightWheels.setIdleMode(IdleMode.kCoast);
    rightWheels.setInverted(false);

    if (ConfigurationProperties.SYS_ID) {
      // Disable PID during SysId
      leftWheels.pidParameters.SetPID(0.0, 0.0, 0.0);
      rightWheels.pidParameters.SetPID(0.0, 0.0, 0.0);
    } else {
      leftWheels.pidParameters.SetPID(0.0001, 0.000001, 0);
      rightWheels.pidParameters.SetPID(0.0001, 0.000001, 0);
    }

    leftWheels.burnFlash();
    rightWheels.burnFlash();
  }

  @Override
  public void periodic() {
    if (ConfigurationProperties.SYS_ID) {
      // Apply the voltage command directly to the motors
      leftWheels.setVoltage(sysIdVoltageCommand);
      rightWheels.setVoltage(sysIdVoltageCommand);
    } else {
      // Internal logging and tuning
      leftWheels.periodic();
      rightWheels.periodic();
    }
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

  // #region SysId

  // Method called by SysId to set the motor voltage
  public void setSysIdVoltageCommand(double voltage) {
    sysIdVoltageCommand = voltage;
  }

  // Method to get the current shooter velocity (in radians per second)
  public double getShooterVelocity() {
    // Assuming the encoder returns velocity in RPM
    double rpm = (leftWheels.encoder.getVelocity() + rightWheels.encoder.getVelocity()) / 2.0;
    return rpm * (2 * Math.PI) / 60.0; // Convert RPM to radians per second
  }

  // #endregion
}
