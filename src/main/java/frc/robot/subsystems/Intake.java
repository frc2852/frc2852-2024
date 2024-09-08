// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanbusId;
import frc.robot.constants.Constants.DIOId;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.hardware.CANSpark;
import frc.robot.util.hardware.CANSpark.MotorModel;

public class Intake extends SubsystemBase {

  // Controllers
  private final CANSpark topRollers = new CANSpark(CanbusId.INTAKE_TOP, MotorModel.VORTEX);
  private final SparkPIDController topRollersPIDController = topRollers.getPIDController();
  private final RelativeEncoder topRollersEncoder = topRollers.getEncoder();
  private PIDParameters topRollersPID;

  private final CANSpark bottomRollers = new CANSpark(CanbusId.INTAKE_BOTTOM, MotorModel.VORTEX);
  private final SparkPIDController bottomRollersPIDController = bottomRollers.getPIDController();
  private final RelativeEncoder bottomRollersEncoder = bottomRollers.getEncoder();
  private PIDParameters bottomRollersPID;

  // Sensors
  private final DigitalInput intakeBeamBreak;
  private final DigitalInput shooterBeamBreak;

  // State
  private boolean isIntakeRunning = false;
  private double velocitySetpoint;

  public Intake() {
    // Set motor controller configurations
    topRollers.setIdleMode(IdleMode.kBrake);
    topRollers.setInverted(false);

    bottomRollers.setIdleMode(IdleMode.kBrake);
    bottomRollers.setInverted(true);

    // Set encoder configurations
    topRollersEncoder.setPosition(0);
    bottomRollersEncoder.setPosition(0);

    // Initialize sensors
    intakeBeamBreak = new DigitalInput(DIOId.INTAKE_BEAM_BREAK);
    shooterBeamBreak = new DigitalInput(DIOId.SHOOTER_BEAM_BREAK);

    // PID coefficients
    topRollersPID = new PIDParameters(
        getName(),
        "TopRollers",
        topRollersPIDController,
        0.0001,
        0.000001,
        0);

    bottomRollersPID = new PIDParameters(
        getName(),
        "BottomRollers",
        bottomRollersPIDController,
        0.0001,
        0.000001,
        0);

    // Save configuration to SparkMax flash
    topRollers.burnFlash();
    bottomRollers.burnFlash();
  }

  @Override
  public void periodic() {
    if (!isGamePieceDeteted() && !isGamePieceLoaded()) {
      runIntakeHalfSpeed();
    } else if (isGamePieceDeteted()) {
      runIntakeFullSpeed();
    } else if (isGamePieceLoaded()) {
      stopIntake();
    }

    updateSmartDashboard();
  }

  private void runIntakeHalfSpeed() {
    velocitySetpoint = MotorSetPoint.INTAKE_VELOCITY;
    topRollersPIDController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollersPIDController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  private void runIntakeFullSpeed() {
    velocitySetpoint = MotorSetPoint.INTAKE_VELOCITY;
    topRollersPIDController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollersPIDController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  private void stopIntake() {
    velocitySetpoint = MotorSetPoint.STOP;
    topRollersPIDController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollersPIDController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  private boolean isGamePieceDeteted() {
    return !intakeBeamBreak.get();
  }

  private boolean isGamePieceLoaded() {
    return !shooterBeamBreak.get();
  }

  private void updateSmartDashboard() {
    topRollersPID.updateSmartDashboard();
    bottomRollersPID.updateSmartDashboard();

    DataTracker.putBoolean(getName(), "IsIntakeRunning", isIntakeRunning);
    DataTracker.putBoolean(getName(), "IsGamePieceLoaded", isGamePieceLoaded());

    if (!DriverStation.isFMSAttached()) {
      // Get current positions and calculate errors
      double topRollersVelocity = topRollersEncoder.getVelocity();
      double topRollersVelocityError = velocitySetpoint - topRollersVelocity;

      double bottomRollersVelocity = bottomRollersEncoder.getVelocity();
      double bottomRollersVelocityError = velocitySetpoint - bottomRollersVelocity;

      DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint);
      DataTracker.putNumber(getName(), "TopRollersVelocity", topRollersVelocity);
      DataTracker.putNumber(getName(), "TopRollersVelocityError", topRollersVelocityError);
      DataTracker.putNumber(getName(), "BottomRollersVelocity", bottomRollersVelocity);
      DataTracker.putNumber(getName(), "BottomRollersVelocityError", bottomRollersVelocityError);
    }
  }
}