package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanbusId;
import frc.robot.util.swerve.SDSMK4iTurnModule;

public class Drive extends SubsystemBase {

  // Create SDSMK4iTurnModules for each wheel
  private final SDSMK4iTurnModule frontLeft = new SDSMK4iTurnModule(
      CanbusId.FRONT_LEFT_TURNING,
      CanbusId.FRONT_LEFT_ENCODER);

  private final SDSMK4iTurnModule frontRight = new SDSMK4iTurnModule(
      CanbusId.FRONT_RIGHT_TURNING,
      CanbusId.FRONT_RIGHT_ENCODER);

  private final SDSMK4iTurnModule rearLeft = new SDSMK4iTurnModule(
      CanbusId.REAR_LEFT_TURNING,
      CanbusId.REAR_LEFT_ENCODER);

  private final SDSMK4iTurnModule rearRight = new SDSMK4iTurnModule(
      CanbusId.REAR_RIGHT_TURNING,
      CanbusId.REAR_RIGHT_ENCODER);

  public Drive() {
    // Initialize default PID values and desired angle
    SmartDashboard.putNumber("PID P", 0.55);
    SmartDashboard.putNumber("PID I", 0.0);
    SmartDashboard.putNumber("PID D", 0.01);
    SmartDashboard.putNumber("DesiredAngle", 90);
  }

  @Override
  public void periodic() {
    // Retrieve PID values from the SmartDashboard
    double p = SmartDashboard.getNumber("PID P", 0.01);
    double i = SmartDashboard.getNumber("PID I", 0.0);
    double d = SmartDashboard.getNumber("PID D", 0.0);

    // Update PID values for each module
    frontLeft.updatePID(p, i, d);
    frontRight.updatePID(p, i, d);
    rearLeft.updatePID(p, i, d);
    rearRight.updatePID(p, i, d);

    // Retrieve the desired angle for each module from the SmartDashboard
    double desiredAngle = SmartDashboard.getNumber("DesiredAngle", 90);
    double inRadians = Math.toRadians(desiredAngle);

    // Set the angle for each module
    frontLeft.setAngle(inRadians);
    frontRight.setAngle(inRadians);
    rearLeft.setAngle(inRadians);
    rearRight.setAngle(inRadians);
  }
}
