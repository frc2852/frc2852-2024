// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;

public class PIDParameters {

  private String groupId;
  private String deviceName;
  private SparkPIDController pidController;

  private double P;
  private double I;
  private double D;

  public double getP() {
    return P;
  }

  public double getI() {
    return I;
  }

  public double getD() {
    return D;
  }

  public PIDParameters(String groupId, String deviceName, SparkPIDController pidController) {
    this.groupId = groupId;
    this.deviceName = deviceName;
    this.pidController = pidController;
  }

  public void SetPID(double P, double I, double D) {
    this.P = P;
    this.I = I;
    this.D = D;

    applyParameters();
    displayParameters();
  }

  public void periodic() {
    if (DriverStation.isFMSAttached())
      return;

    boolean pendingPIDUpdate = false;
    double newP = DataTracker.getNumber(groupId, deviceName, "P", P);
    if (newP != P) {
      P = newP;
      pendingPIDUpdate = true;
    }

    double newI = DataTracker.getNumber(groupId, deviceName, "I", I);
    if (newI != I) {
      I = newI;
      pendingPIDUpdate = true;
    }

    double newD = DataTracker.getNumber(groupId, deviceName, "D", D);
    if (newD != D) {
      D = newD;
      pendingPIDUpdate = true;
    }

    if (pendingPIDUpdate) {
      applyParameters();
    }
  }

  private void applyParameters() {
    if (validatePIDValues(P, I, D)) {
      this.pidController.setP(P);
      this.pidController.setI(I);
      this.pidController.setD(D);
    } else {
      DriverStation.reportError("Invalid PID values: P, I, and D must be non-negative.", false);
    }
  }

  private boolean validatePIDValues(double P, double I, double D) {
    return (P >= 0 && I >= 0 && D >= 0);
  }

  private void displayParameters() {
    if (DriverStation.isFMSAttached())
      return;

    DataTracker.putNumber(groupId, deviceName, "P", P);
    DataTracker.putNumber(groupId, deviceName, "I", I);
    DataTracker.putNumber(groupId, deviceName, "D", D);
  }
}
