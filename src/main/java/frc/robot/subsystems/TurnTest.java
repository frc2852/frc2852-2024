// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.util.testing.SDSMK4iTurn;

public class TurnTest extends SubsystemBase {

  // Create MAXSwerveModules
  private final SDSMK4iTurn frontLeft = new SDSMK4iTurn(CANBus.FRONT_LEFT_TURN, CANBus.FRONT_LEFT_ENCODER, true);
  private final SDSMK4iTurn frontRight = new SDSMK4iTurn(CANBus.FRONT_RIGHT_TURN, CANBus.FRONT_RIGHT_ENCODER, true);
  private final SDSMK4iTurn rearLeft = new SDSMK4iTurn(CANBus.REAR_LEFT_TURN, CANBus.REAR_LEFT_ENCODER, true);
  private final SDSMK4iTurn rearRight = new SDSMK4iTurn(CANBus.REAR_RIGHT_TURN, CANBus.REAR_RIGHT_ENCODER, true);

  public TurnTest() {
    SmartDashboard.putNumber("SetPosition", 0);

    SmartDashboard.putNumber("P", 0.55 );
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0.01);

  }

  @Override
  public void periodic() {
    updatePID();

    double position = SmartDashboard.getNumber("SetPosition", 0);
    var radiansPosition = Math.toRadians(position);
    frontLeft.setAngle(radiansPosition);
    frontRight.setAngle(radiansPosition);
    rearLeft.setAngle(radiansPosition);
    rearRight.setAngle(radiansPosition);
  }

  private void updatePID() {
    var p = SmartDashboard.getNumber("P", 0);
    var i = SmartDashboard.getNumber("I", 0);
    var d = SmartDashboard.getNumber("D", 0);

    frontLeft.updatePID(p, i, d);
    frontRight.updatePID(p, i, d);
    rearLeft.updatePID(p, i, d);
    rearRight.updatePID(p, i, d);
  }
}