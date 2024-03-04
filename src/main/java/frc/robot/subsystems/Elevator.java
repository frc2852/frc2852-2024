// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.Delay;
import frc.robot.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class Elevator extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkPIDController motorPID;
  private final RelativeEncoder motorEncoder;
  private PIDParameters motorPidParameters;

  private double positionSetpoint = MotorSetpoint.ELEVATOR_DRIVE_POSITION;

  private boolean updateMotorPID = false;

  public Elevator() {

    // Initialize motor controllers
    motor = new SparkFlex(CanbusId.ELEVATOR, Delay.ELEVATOR);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);

    // Initialize PID controllers
    motorPID = motor.getPIDController();
    motorEncoder = motor.getEncoder();

    // Zero encoder position
    motorEncoder.setPosition(0);

    // PID coefficients
    motorPidParameters = new PIDParameters(
        getName(),
        "",
        0.1, 0, 0, 0, 0, -MotorSetpoint.ELEVAOTOR_MAX_OUPUT, MotorSetpoint.ELEVAOTOR_MAX_OUPUT);

    // set PID coefficients
    motorPidParameters.applyParameters(motorPID);

    // Save configuration to SparkMax flash
    motor.burnFlash();

    // Add update buttons to dashboard
    DataTracker.putBoolean(getName(), "UpdatePID", updateMotorPID, true);
  }

  @Override
  public void periodic() {

  }

  public void drivePosition() {
    if (positionSetpoint != MotorSetpoint.ELEVATOR_DRIVE_POSITION) {
      positionSetpoint = MotorSetpoint.ELEVATOR_DRIVE_POSITION;
      moveElevatorToPosition();
    }
  }

  public void ampPosition() {
    if (positionSetpoint != MotorSetpoint.ELEVATOR_AMP_POSITION) {
      positionSetpoint = MotorSetpoint.ELEVATOR_AMP_POSITION;
      moveElevatorToPosition();
    }
  }

  public void climbPosition() {
    if (positionSetpoint != MotorSetpoint.ELEVATOR_CLIMB_POSITION) {
      positionSetpoint = MotorSetpoint.ELEVATOR_CLIMB_POSITION;
      moveElevatorToPosition();
    }
  }

  public void trapPosition() {
    if (positionSetpoint != MotorSetpoint.ELEVATOR_TRAP_POSITION) {
      positionSetpoint = MotorSetpoint.ELEVATOR_TRAP_POSITION;
      moveElevatorToPosition();
    }
  }

  public boolean isElevatorAtPosition() {
    return Math.abs(motorEncoder.getPosition() - positionSetpoint) < MotorSetpoint.ELEVATOR_MARGIN_OF_ERROR;
  }

  private void moveElevatorToPosition() {
    motorPID.setReference(positionSetpoint, CANSparkMax.ControlType.kPosition);
  }
}