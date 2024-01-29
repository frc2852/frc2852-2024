// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.DIOId;
import frc.robot.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex topRoller;
  private final SparkPIDController topRollerPID;
  private final RelativeEncoder topRollerEncoder;
  private PIDParameters topRollerPidParameters;

  private final SparkFlex bottomRoller;
  private final SparkPIDController bottomRollerPID;
  private final RelativeEncoder bottomRollerEncoder;
  private PIDParameters bottomRollerPidParameters;

  private double velocitySetpoint;

  private boolean updateTopRollerPID = false;
  private boolean updateBottomRollerPID = false;

  private final DigitalInput shooterProximitySensor;

  public ShooterSubsystem() {

    // Initialize motor controllers
    topRoller = new SparkFlex(CanbusId.SHOOTER_TOP_ROLLER);
    topRoller.setIdleMode(IdleMode.kCoast);
    topRoller.setInverted(false);

    bottomRoller = new SparkFlex(CanbusId.SHOOTER_BOTTOM_ROLLER);
    bottomRoller.setIdleMode(IdleMode.kCoast);
    bottomRoller.setInverted(true);

    // Initialize proximity sensors
    shooterProximitySensor = new DigitalInput(DIOId.SHOOTER_PROXIMITY_SENSOR);

    // Initialize PID controllers
    topRollerPID = topRoller.getPIDController();
    topRollerEncoder = topRoller.getEncoder();

    bottomRollerPID = bottomRoller.getPIDController();
    bottomRollerEncoder = bottomRoller.getEncoder();

    // PID coefficients
    topRollerPidParameters = new PIDParameters(
        getName(),
        "TopRoller",
        0, 0, 0, 0, 0, -1, 1);

    bottomRollerPidParameters = new PIDParameters(
        getName(),
        "BottomRoller",
        0, 0, 0, 0, 0, -1, 1);

    // set PID coefficients
    topRollerPidParameters.applyParameters(topRollerPID);
    bottomRollerPidParameters.applyParameters(bottomRollerPID);

    // Save configuration to SparkMax flash
    topRoller.burnFlash();
    bottomRoller.burnFlash();

    // Add update buttons to dashboard
    DataTracker.putBoolean(getName(), "UpdateTopRollerPID", updateTopRollerPID, true);
    DataTracker.putBoolean(getName(), "UpdateBottomRollerPID", updateBottomRollerPID, true);
  }

  @Override
  public void periodic() {
    // Get current positions and calculate errors
    double topRollerVelocity = topRollerEncoder.getVelocity();
    double topRollerVelocityError = velocitySetpoint == 0 ? 0 : topRollerVelocity - velocitySetpoint;

    double bottomRollerVelocity = bottomRollerEncoder.getVelocity();
    double bottomRollerVelocityError = velocitySetpoint == 0 ? 0 : topRollerVelocity - velocitySetpoint;

    // Dashboard data tracking
    DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);

    DataTracker.putNumber(getName(), "TopRollerVelocity", topRollerVelocity, true);
    DataTracker.putNumber(getName(), "TopRollerVelocityError", topRollerVelocityError, true);

    DataTracker.putNumber(getName(), "BottomRollerVelocity", bottomRollerVelocity, true);
    DataTracker.putNumber(getName(), "BottomRollerVelocityError", bottomRollerVelocityError, true);

    if (!DriverStation.isFMSAttached() && Constants.PID_TUNE_MODE) {

      // PID updates from dashboard
      updateTopRollerPID = SmartDashboard.getBoolean("UpdateTopRollerPID", false);
      updateBottomRollerPID = SmartDashboard.getBoolean("UpdateBottomRollerPID", false);

      if (topRollerPidParameters.updateParametersFromDashboard() && updateTopRollerPID) {
        updateTopRollerPID = false;
        topRollerPidParameters.applyParameters(topRollerPID);
      }

      if (bottomRollerPidParameters.updateParametersFromDashboard() && updateBottomRollerPID) {
        updateBottomRollerPID = false;
        bottomRollerPidParameters.applyParameters(bottomRollerPID);
      }
    }
  }

  public void divertGamePiece() {
    setShooterSpeed(-MotorSetpoint.SHOOTER_DIVERT_VELOCITY);
  }

  public void shootGamePiece() {
    setShooterSpeed(MotorSetpoint.SHOOTER_VELOCITY);
  }

  public void stopShooter() {
    velocitySetpoint = 0;
    topRoller.stopMotor();
    bottomRoller.stopMotor();
  }

  public boolean isShooterAtSpeed() {
    double topRollerVelocity = topRollerEncoder.getVelocity();
    double bottomRollerVelocity = bottomRollerEncoder.getVelocity();
    return (Math.abs(topRollerVelocity - velocitySetpoint) < MotorSetpoint.SHOOTER_MARGIN_OF_ERROR && Math.abs(bottomRollerVelocity - velocitySetpoint) < MotorSetpoint.SHOOTER_MARGIN_OF_ERROR);

  }

  public boolean isGamePieceShot() {
    return !shooterProximitySensor.get();
  }

  private void setShooterSpeed(double velocity) {
    velocitySetpoint = velocity;
    topRollerPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    bottomRollerPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }
}
