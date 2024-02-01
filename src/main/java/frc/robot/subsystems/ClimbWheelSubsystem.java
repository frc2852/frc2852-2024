// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class ClimbWheelSubsystem extends SubsystemBase {

  private final SparkFlex climbWheelMotor;
  private final SparkPIDController climbWheelPID;
  private final RelativeEncoder climbWheelEncoder;
  private PIDParameters climbWheelPidParameters;

  private boolean updateClimbWheelsPID = false;
  private double velocitySetpoint;

  public ClimbWheelSubsystem() {
    // Initialize climb wheel motor
    climbWheelMotor = new SparkFlex(CanbusId.CLIMB_WHEELS);
    climbWheelMotor.setIdleMode(IdleMode.kBrake);
    climbWheelMotor.setInverted(false);

    // Initialize PID controller and encoder
    climbWheelPID = climbWheelMotor.getPIDController();
    climbWheelEncoder = climbWheelMotor.getEncoder();

    // PID coefficients (these values should be tuned for your specific system)
    climbWheelPidParameters = new PIDParameters(
        getName(),
        "",
        0.0001, 0.000001, 0.0, 0.0, 0.0, -1.0, 1.0);

    // Apply PID parameters
    climbWheelPidParameters.applyParameters(climbWheelPID);

    // Save configuration to flash
    climbWheelMotor.burnFlash();

    // Add update buttons to dashboard
    DataTracker.putBoolean(getName(), "UpdatePID", updateClimbWheelsPID, true);
  }

  @Override
  public void periodic() {
    // Get current positions and calculate errors
    double climbWheelVelocity = climbWheelEncoder.getVelocity();
    double climbWheelVelocityError = MotorSetpoint.CLIMB_WHEEL_VELOCITY - climbWheelVelocity;

    // Dashboard data tracking
    DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);
    DataTracker.putNumber(getName(), "Velocity", climbWheelVelocity, true);
    DataTracker.putNumber(getName(), "VelocityError", climbWheelVelocityError, true);

    if (!DriverStation.isFMSAttached() && Constants.PID_TUNE_MODE) {

      // PID updates from dashboard
      updateClimbWheelsPID = SmartDashboard.getBoolean("UpdatePID", false);

      if (climbWheelPidParameters.updateParametersFromDashboard() && updateClimbWheelsPID) {
        updateClimbWheelsPID = false;
        climbWheelPidParameters.applyParameters(climbWheelPID);
      }
    }
  }

  public void runClimbWheels() {
    velocitySetpoint = MotorSetpoint.CLIMB_WHEEL_VELOCITY;
    climbWheelPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stopClimbWheels() {
    velocitySetpoint = 0;
    climbWheelPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }
}
