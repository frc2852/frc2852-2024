// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.DIOId;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.hardware.SparkFlex;

public class Intake extends SubsystemBase {

  // Controllers
  private final SparkFlex topRollers = new SparkFlex(CANBus.INTAKE_TOP);
  private final SparkFlex bottomRollers = new SparkFlex(CANBus.INTAKE_LOWER);

  // Sensors
  private final DigitalInput intakeBeamBreak;
  private final DigitalInput shooterBeamBreak;

  // State
  private double velocitySetpoint;
  private boolean gamePieceLoading;

  public Intake() {
    // Set motor controller configurations
    topRollers.setIdleMode(IdleMode.kBrake);
    topRollers.setInverted(true);
    topRollers.pidParameters.SetPID(0.0001, 0.000001, 0);
    topRollers.burnFlash();

    bottomRollers.setIdleMode(IdleMode.kBrake);
    bottomRollers.setInverted(true);
    bottomRollers.pidParameters.SetPID(0.0001, 0.000001, 0);
    bottomRollers.burnFlash();

    // Initialize sensors
    intakeBeamBreak = new DigitalInput(DIOId.INTAKE_BEAM_BREAK);
    shooterBeamBreak = new DigitalInput(DIOId.SHOOTER_BEAM_BREAK);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isGamePieceDeteted", isGamePieceDeteted());
    SmartDashboard.putBoolean("isGamePieceLoaded", isGamePieceLoaded());
    
    if (!isGamePieceDeteted() && !isGamePieceLoaded() && !isGamePieceBeingLoaded()) {
      runIntakeHalfSpeed();
    } else if (isGamePieceDeteted()) {
      runIntakeFullSpeed();
      gamePieceLoading = true;
    } else if (isGamePieceLoaded()) {
      stopIntake();
      gamePieceLoading = false;
    }

    topRollers.periodic();
    bottomRollers.periodic();
  }

  private void runIntakeHalfSpeed() {
    velocitySetpoint = MotorSetPoint.INTAKE_HALF;
    topRollers.setVelocity(0);
    bottomRollers.setVelocity(velocitySetpoint);
  }

  private void runIntakeFullSpeed() {
    velocitySetpoint = MotorSetPoint.INTAKE_FULL;
    topRollers.setVelocity(velocitySetpoint);
    bottomRollers.setVelocity(velocitySetpoint);
  }

  private void stopIntake() {
    velocitySetpoint = MotorSetPoint.STOP;
    topRollers.setVelocity(velocitySetpoint);
    bottomRollers.setVelocity(velocitySetpoint);
  }

  private boolean isGamePieceBeingLoaded() {
    return gamePieceLoading;
  }

  private boolean isGamePieceDeteted() {
    return !intakeBeamBreak.get();
  }

  private boolean isGamePieceLoaded() {
    return !shooterBeamBreak.get();
  }
}