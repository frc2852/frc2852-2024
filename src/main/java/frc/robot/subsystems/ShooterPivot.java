package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.MotorProperties;
import frc.robot.constants.Constants.MotorSetPoint;

public class ShooterPivot extends SubsystemBase {

  // Controllers
  private final CANSparkFlex pivot = new CANSparkFlex(CANBus.SHOOTER_PIVOT.getCanId(), MotorType.kBrushless);
  private final SparkPIDController pid;
  private final RelativeEncoder encoder;
  // State
  private double positionSetpoint;

  public ShooterPivot() {
    // Set motor controller configurations
    pivot.setIdleMode(IdleMode.kBrake);
    pivot.setInverted(true);

    pid = pivot.getPIDController();
    pid.setP(0.0001);
    pid.setI(0.000001);
    pid.setD(0);

    pid.setSmartMotionMaxVelocity(5000, 0);
    pid.setSmartMotionMaxAccel(800, 0);

    encoder = pivot.getEncoder();
    encoder.setPosition(0);
    pid.setFeedbackDevice(encoder);

    pivot.burnFlash();

    SmartDashboard.putNumber("PivotSetPoint", 0);
  }

  @Override
  public void periodic() {

  }

  // Pivot laying on robot
  public void pivotLoadPosition() {
    positionSetpoint = MotorSetPoint.PIVOT_LOAD;
    pid.setSmartMotionMaxVelocity(MotorProperties.VORTEX_MAX_RPM, 0);
    pid.setSmartMotionMaxAccel(800, 0);
    pid.setReference(positionSetpoint, ControlType.kSmartMotion);
  }

  // Pivot raised into the air
  public void pivotShootPosition() {
    positionSetpoint = MotorSetPoint.PIVOT_SHOOT;
    pid.setSmartMotionMaxVelocity(MotorProperties.VORTEX_MAX_RPM, 0);
    pid.setSmartMotionMaxAccel(3000, 0);
    pid.setReference(positionSetpoint, ControlType.kSmartMotion);
  }
}
