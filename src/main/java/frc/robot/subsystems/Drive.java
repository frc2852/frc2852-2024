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
    // Initialize default angles for all modules on the SmartDashboard
    SmartDashboard.putNumber("DesiredAngle", 90);
  }

  @Override
  public void periodic() {
    // Retrieve the desired angle for each module from the SmartDashboard
    double desiredAngle = SmartDashboard.getNumber("DesiredAngle", 90);

    // Set the angle for each module
    frontLeft.setAngle(desiredAngle);
    frontRight.setAngle(desiredAngle);
    rearLeft.setAngle(desiredAngle);
    rearRight.setAngle(desiredAngle);
  }
}
