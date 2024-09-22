package frc.robot.util;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;

public class PIDParameters {

  private String groupId;
  private String deviceName;
  private SparkPIDController pidController;

  private Double P;
  private Double I;
  private Double D;
  private Double FF;

  public Double getP() {
    return P;
  }

  public Double getI() {
    return I;
  }

  public Double getD() {
    return D;
  }

  public Double getFF() {
    return FF;
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
    this.FF = null;

    applyParameters();
    displayParameters();
  }

  public void SetPID(double P, double I, double D, double FF) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.FF = FF;

    applyParameters();
    displayParameters();
  }

  public void periodic() {
    if (DriverStation.isFMSAttached())
      return;

    boolean pendingPIDUpdate = false;
    Double newP = DataTracker.getNumber(groupId, deviceName, "P", P);
    if (newP != P) {
      P = newP;
      pendingPIDUpdate = true;
    }

    Double newI = DataTracker.getNumber(groupId, deviceName, "I", I);
    if (newI != I) {
      I = newI;
      pendingPIDUpdate = true;
    }

    Double newD = DataTracker.getNumber(groupId, deviceName, "D", D);
    if (newD != D) {
      D = newD;
      pendingPIDUpdate = true;
    }

    if (FF != null) {
      Double newFF = DataTracker.getNumber(groupId, deviceName, "FF", FF);
      if (newFF != FF) {
        FF = newFF;
        pendingPIDUpdate = true;
      }
    }

    if (pendingPIDUpdate) {
      applyParameters();
    }
  }

  private void applyParameters() {
    if (validatePIDValues(P, I, D, FF)) {
      this.pidController.setP(P);
      this.pidController.setI(I);
      this.pidController.setD(D);

      if (this.FF != null) {
        this.pidController.setFF(FF);
      }

    } else {
      DriverStation.reportError("Invalid PID values: P, I, and D must be non-negative.", false);
    }
  }

  private boolean validatePIDValues(Double P, Double I, Double D, Double FF) {
    return (P >= 0 && I >= 0 && D >= 0 && (FF == null || FF >= 0));
  }

  private void displayParameters() {
    if (DriverStation.isFMSAttached())
      return;

    DataTracker.putNumber(groupId, deviceName, "P", P);
    DataTracker.putNumber(groupId, deviceName, "I", I);
    DataTracker.putNumber(groupId, deviceName, "D", D);
    DataTracker.putNumber(groupId, deviceName, "FF", FF);
  }
}
