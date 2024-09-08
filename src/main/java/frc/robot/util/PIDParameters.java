// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDParameters {

  private String groupId;
  private String namePrefix;
  private double P;
  private double I;
  private double D;
  private SparkPIDController pidController;

  public double getP() {
    return P;
  }

  public double getI() {
    return I;
  }

  public double getD() {
    return D;
  }

  public void setP(double kP) {
    this.P = kP;
  }

  public void setI(double kI) {
    this.I = kI;
  }

  public void setKD(double kD) {
    this.D = kD;
  }

  public PIDParameters(String groupId, String namePrefix, SparkPIDController pidController, double P, double I, double D) {
    this.groupId = groupId;
    this.namePrefix = namePrefix;
    this.pidController = pidController;
    this.P = P;
    this.I = I;
    this.D = D;

    applyParameters();
    displayParameters();
  }

  public void updateSmartDashboard() {
    if (DriverStation.isFMSAttached())
      return;

    boolean pendingPIDUpdate = false;
    double newP = DataTracker.getNumber(groupId, namePrefix + "P", P);
    if (newP != P) {
      P = newP;
      pendingPIDUpdate = true;
    }

    double newI = DataTracker.getNumber(groupId, namePrefix + "I", I);
    if (newI != I) {
      I = newI;
      pendingPIDUpdate = true;
    }

    double newD = DataTracker.getNumber(groupId, namePrefix + "D", D);
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

    DataTracker.putNumber(groupId, namePrefix + "P", P);
    DataTracker.putNumber(groupId, namePrefix + "I", I);
    DataTracker.putNumber(groupId, namePrefix + "D", D);
  }
}
