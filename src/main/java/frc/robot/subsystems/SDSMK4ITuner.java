package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.util.swerve.SDSMK4iModuleTune;

public class SDSMK4ITuner extends SubsystemBase {

  // Create MAXSwerveModules
  private final SDSMK4iModuleTune frontLeft = new SDSMK4iModuleTune(CANBus.FRONT_LEFT_TURN, CANBus.FRONT_LEFT_ENCODER, true);
  private final SDSMK4iModuleTune frontRight = new SDSMK4iModuleTune(CANBus.FRONT_RIGHT_TURN, CANBus.FRONT_RIGHT_ENCODER, true);
  private final SDSMK4iModuleTune rearLeft = new SDSMK4iModuleTune(CANBus.REAR_LEFT_TURN, CANBus.REAR_LEFT_ENCODER, true);
  private final SDSMK4iModuleTune rearRight = new SDSMK4iModuleTune(CANBus.REAR_RIGHT_TURN, CANBus.REAR_RIGHT_ENCODER, true);

  public SDSMK4ITuner() {
    SmartDashboard.putNumber("SetPosition", 0);

    SmartDashboard.putNumber("P", 0.620000 );
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0.000001);

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