package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.MotorProperties;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.NoteTracker;

public class ShooterPivot extends SubsystemBase {

  // Controllers
  private final CANSparkFlex pivot;
  private final SparkPIDController pid;
  private final RelativeEncoder encoder;
  private final NoteTracker noteTracker;

  // State
  private double positionSetpoint;
  private ControlType currentControlType = ControlType.kSmartMotion;
  private static final double POSITION_TOLERANCE = 0.5;

  private double p = 0.000170;
  private double i = 0.000001;
  private double d = 0.000010;

  public ShooterPivot(NoteTracker noteTracker) {

    this.noteTracker = noteTracker;

    // Initialize motor controller
    pivot = new CANSparkFlex(CANBus.SHOOTER_PIVOT.getCanId(), MotorType.kBrushless);

    // Set motor controller configurations
    pivot.setIdleMode(IdleMode.kBrake);
    pivot.setInverted(true);

    pid = pivot.getPIDController();
    pid.setP(p);
    pid.setI(i);
    pid.setD(d);
    
    pid.setSmartMotionAllowedClosedLoopError(POSITION_TOLERANCE, 0); // Set allowable error

    encoder = pivot.getEncoder();
    encoder.setPosition(0);
    pid.setFeedbackDevice(encoder);

    pivot.setSmartCurrentLimit(40);
    pivot.burnFlash();

    // Add default values for PID to the SmartDashboard
    // SmartDashboard.putNumber("ShooterPivot P", p);
    // SmartDashboard.putNumber("ShooterPivot I", i);
    // SmartDashboard.putNumber("ShooterPivot D", d);
  }

  @Override
  public void periodic() {
    // // Update PID values from SmartDashboard
    // double _p = SmartDashboard.getNumber("ShooterPivot P", p);
    // double _i = SmartDashboard.getNumber("ShooterPivot I", i);
    // double _d = SmartDashboard.getNumber("ShooterPivot D", d);

    // // Set the updated PID values
    // pid.setP(_p);
    // pid.setI(_i);
    // pid.setD(_d);

    // Check if we are at the setpoint
    if (isAtSetpoint()) {
      if (positionSetpoint == MotorSetPoint.PIVOT_LOAD) {
        // At load position, set motor output to zero to prevent pushing against the stop
        if (currentControlType != ControlType.kDutyCycle) {
          pid.setReference(0, ControlType.kDutyCycle);
          currentControlType = ControlType.kDutyCycle;
        }
      }
    }
  }

  // Pivot to loading position
  public void pivotLoadPosition() {
    positionSetpoint = MotorSetPoint.PIVOT_LOAD;
    pid.setSmartMotionMaxVelocity(3000, 0);
    pid.setSmartMotionMaxAccel(1500, 0);
    pid.setReference(positionSetpoint, ControlType.kSmartMotion);
    currentControlType = ControlType.kSmartMotion;
  }

  // Pivot to shooting position
  public void pivotShootPosition() {
    if (noteTracker.hasNote()) {
      positionSetpoint = MotorSetPoint.PIVOT_SHOOT;
      pid.setSmartMotionMaxVelocity(MotorProperties.VORTEX_MAX_RPM, 0);
      pid.setSmartMotionMaxAccel(MotorProperties.VORTEX_MAX_RPM, 0);
      pid.setReference(positionSetpoint, ControlType.kSmartMotion);
      currentControlType = ControlType.kSmartMotion;
    }
  }

  public void pivotShootLongPosition() {
    if (noteTracker.hasNote()) {
      positionSetpoint = MotorSetPoint.PIVOT__LONG_SHOOT;
      pid.setSmartMotionMaxVelocity(MotorProperties.VORTEX_MAX_RPM, 0);
      pid.setSmartMotionMaxAccel(MotorProperties.VORTEX_MAX_RPM, 0);
      pid.setReference(positionSetpoint, ControlType.kSmartMotion);
      currentControlType = ControlType.kSmartMotion;
    }
  }

  // Check if the pivot has reached the setpoint
  public boolean isAtSetpoint() {
    double currentPosition = encoder.getPosition();
    return Math.abs(currentPosition - positionSetpoint) <= POSITION_TOLERANCE;
  }
}
