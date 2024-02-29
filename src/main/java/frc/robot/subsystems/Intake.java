// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.DIOId;
import frc.robot.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class Intake extends SubsystemBase {

  private final SparkFlex topRollers = new SparkFlex(CanbusId.INTAKE_TOP_ROLLER);
  private final SparkPIDController topRollersPID = topRollers.getPIDController();
  private final RelativeEncoder topRollersEncoder = topRollers.getEncoder();
  private PIDParameters topRollersPidParameters;

  private final SparkFlex bottomRollers = new SparkFlex(CanbusId.INTAKE_BOTTOM_ROLLER);
  private final SparkPIDController bottomRollersPID = bottomRollers.getPIDController();
  private final RelativeEncoder bottomRollersEncoder = bottomRollers.getEncoder();
  private PIDParameters bottomRollersPidParameters;

  private final DigitalInput intakeBeamBreak;

  private boolean isIntakeRunning = false;
  private boolean isConveyorMode = false;
  private double velocitySetpoint;

  private final double INTAKE_STOPPED_VELOCITY = 0.0;

  public Intake() {

    // Set motor controller configurations
    topRollers.setIdleMode(IdleMode.kBrake);
    topRollers.setInverted(false);

    bottomRollers.setIdleMode(IdleMode.kBrake);
    bottomRollers.setInverted(true);

    // Set encoder configurations
    topRollersEncoder.setPosition(0);
    bottomRollersEncoder.setPosition(0);

    // Initialize proximity sensors
    intakeBeamBreak = new DigitalInput(DIOId.INTAKE_BEAM_BREAK);

    // PID coefficients
    topRollersPidParameters = new PIDParameters(
        getName(),
        "TopRollers",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    bottomRollersPidParameters = new PIDParameters(
        getName(),
        "BottomRollers",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    // Set PID coefficients
    topRollersPidParameters.applyParameters(topRollersPID);
    bottomRollersPidParameters.applyParameters(bottomRollersPID);

    // Save configuration to SparkMax flash
    topRollers.burnFlash();
    bottomRollers.burnFlash();
  }

  @Override
  public void periodic() {
    // Automatically stop intake if game piece is loaded and we are not in conveyor mode
    if (isGamePieceLoaded() && isIntakeRunning && !isConveyorMode) {
      isConveyorMode = true;
      stopIntake();
    }
    UpdateDataTracking();
  }

  public void toggleIntake() {
    isIntakeRunning = !isIntakeRunning;
    if (isIntakeRunning) {
      runIntake(false);
    } else {
      stopIntake();
    }
  }

  public void runIntake(boolean conveyorMode) {
    isIntakeRunning = true;
    isConveyorMode = conveyorMode;

    velocitySetpoint = MotorSetpoint.INTAKE_VELOCITY;
    topRollersPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollersPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stopIntake() {
    isIntakeRunning = false;
    velocitySetpoint = INTAKE_STOPPED_VELOCITY;
    topRollersPID.setReference(INTAKE_STOPPED_VELOCITY, CANSparkMax.ControlType.kVelocity);
    bottomRollersPID.setReference(INTAKE_STOPPED_VELOCITY, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isGamePieceLoaded() {
    return !intakeBeamBreak.get();
  }

  private void UpdateDataTracking() {
    // Get current positions and calculate errors
    double topRollersVelocity = topRollersEncoder.getVelocity();
    double topRollersVelocityError = velocitySetpoint - topRollersVelocity;

    double bottomRollersVelocity = bottomRollersEncoder.getVelocity();
    double bottomRollersVelocityError = velocitySetpoint - bottomRollersVelocity;

    DataTracker.putBoolean(getName(), "IsRunning", isIntakeRunning, true);
    DataTracker.putBoolean(getName(), "IsGamePieceLoaded", isGamePieceLoaded(), true);

    DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);
    DataTracker.putNumber(getName(), "TopRollersVelocity", topRollersVelocity, true);
    DataTracker.putNumber(getName(), "TopRollersVelocityError", topRollersVelocityError, true);
    DataTracker.putNumber(getName(), "BottomRollersVelocity", bottomRollersVelocity, true);
    DataTracker.putNumber(getName(), "BottomRollersVelocityError", bottomRollersVelocityError, true);
  }
}