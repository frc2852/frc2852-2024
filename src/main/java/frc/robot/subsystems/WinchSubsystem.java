// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class WinchSubsystem extends SubsystemBase {

  private final SparkFlex leftWinchMotor;
  private final SparkPIDController leftWinchMotorPID;
  private final RelativeEncoder leftWinchMotorEncoder;
  private PIDParameters leftWinchMotorPidParameters;

  private final SparkFlex rightWinchMotor;
  private final SparkPIDController rightWinchMotorPID;
  private final RelativeEncoder rightWinchMotorEncoder;
  private PIDParameters rightWinchMotorPidParameters;

  private double positionSetpoint;

  private boolean updateLeftWinchMotorPID = false;
  private boolean updateRightWinchMotorPID = false;

  public WinchSubsystem() {
    // Initialize motor controllers
    leftWinchMotor = new SparkFlex(CanbusId.WINCH_LEFT);
    leftWinchMotor.setIdleMode(IdleMode.kBrake);
    leftWinchMotor.setInverted(false);

    rightWinchMotor = new SparkFlex(CanbusId.WINCH_RIGHT);
    rightWinchMotor.setIdleMode(IdleMode.kBrake);
    rightWinchMotor.setInverted(false);

    // Initialize PID controllers
    leftWinchMotorPID = leftWinchMotor.getPIDController();
    leftWinchMotorEncoder = leftWinchMotor.getEncoder();

    rightWinchMotorPID = rightWinchMotor.getPIDController();
    rightWinchMotorEncoder = rightWinchMotor.getEncoder();

    // Zero encoders
    leftWinchMotorEncoder.setPosition(0);
    rightWinchMotorEncoder.setPosition(0);

    // PID coefficients
    leftWinchMotorPidParameters = new PIDParameters(
        getName(),
        "LeftMotor",
        0.1, 0, 0, 0, 0, -MotorSetpoint.ELEVAOTOR_MAX_OUPUT, MotorSetpoint.ELEVAOTOR_MAX_OUPUT);

    rightWinchMotorPidParameters = new PIDParameters(
        getName(),
        "RightMotor",
        0.1, 0, 0, 0, 0, -MotorSetpoint.ELEVAOTOR_MAX_OUPUT, MotorSetpoint.ELEVAOTOR_MAX_OUPUT);

    // Set PID coefficients
    leftWinchMotorPidParameters.applyParameters(leftWinchMotorPID);
    rightWinchMotorPidParameters.applyParameters(rightWinchMotorPID);

    // Save configuration to SparkMax flash
    leftWinchMotor.burnFlash();
    rightWinchMotor.burnFlash();

    // Add update buttons to dashboard
    DataTracker.putBoolean(getName(), "UpdateLeftMotorPID", updateLeftWinchMotorPID, true);
    DataTracker.putBoolean(getName(), "UpdateRightMotorPID", updateRightWinchMotorPID, true);
  }

  @Override
  public void periodic() {
    // Get current positions and calculate errors
    double leftWinchMotorPosition = leftWinchMotorEncoder.getPosition();
    double leftWinchMotorPositionError = positionSetpoint - leftWinchMotorPosition;

    double rightWinchMotorPosition = rightWinchMotorEncoder.getPosition();
    double rightWinchMotorPositionError = positionSetpoint - rightWinchMotorPosition;

    // Dashboard data tracking
    DataTracker.putNumber(getName(), "PositionSetPoint", positionSetpoint, true);
    DataTracker.putNumber(getName(), "LeftMotorPosition", leftWinchMotorPosition, true);
    DataTracker.putNumber(getName(), "LeftMotorPositionError", leftWinchMotorPositionError, true);
    DataTracker.putNumber(getName(), "RightMotorPosition", rightWinchMotorPosition, true);
    DataTracker.putNumber(getName(), "RightMotorPositionError", rightWinchMotorPositionError, true);

    // PID updates from dashboard
    if (!DriverStation.isFMSAttached() && Constants.PID_TUNE_MODE) {

      // PID updates from dashboard
      updateLeftWinchMotorPID = SmartDashboard.getBoolean("UpdateLeftMotorPID", false);
      updateRightWinchMotorPID = SmartDashboard.getBoolean("UpdateRightMotorPID", false);

      if (leftWinchMotorPidParameters.updateParametersFromDashboard() && updateLeftWinchMotorPID) {
        updateLeftWinchMotorPID = false;
        leftWinchMotorPidParameters.applyParameters(leftWinchMotorPID);
      }

      if (rightWinchMotorPidParameters.updateParametersFromDashboard() && updateRightWinchMotorPID) {
        updateRightWinchMotorPID = false;
        rightWinchMotorPidParameters.applyParameters(rightWinchMotorPID);
      }
    }
  }

  public void armsUp() {
    setWinchPosition(MotorSetpoint.WINCH_ARMS_UP_POSITION);
  }

  public void armsDown() {
    setWinchPosition(MotorSetpoint.WINCH_ARMS_DOWN_POSITION);
  }

  public boolean areArmsAtPosition() {
    return Math.abs(leftWinchMotorEncoder.getPosition() - positionSetpoint) < MotorSetpoint.WINCH_ARMS_MARGIN_OF_ERROR;
  }

  private void setWinchPosition(double position) {
    positionSetpoint = position;
    leftWinchMotorPID.setReference(position, CANSparkMax.ControlType.kPosition);
    rightWinchMotorPID.setReference(position, CANSparkMax.ControlType.kPosition);
  }
}
