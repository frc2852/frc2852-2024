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

  public ShooterPivot(NoteTracker noteTracker) {

    this.noteTracker = noteTracker;

    // Initialize motor controller
    pivot = new CANSparkFlex(CANBus.SHOOTER_PIVOT.getCanId(), MotorType.kBrushless);

    // Set motor controller configurations
    pivot.setIdleMode(IdleMode.kBrake);
    pivot.setInverted(false);

    pid = pivot.getPIDController();
    pid.setP(0.0001);
    pid.setI(0); //Eliminate
    pid.setD(0);

    pid.setSmartMotionAllowedClosedLoopError(POSITION_TOLERANCE, 0); // Set allowable error

    pid.setSmartMotionMaxVelocity(5000, 0);
    pid.setSmartMotionMaxAccel(800, 0);

    encoder = pivot.getEncoder();
    encoder.setPosition(0);
    pid.setFeedbackDevice(encoder);

    pivot.setSmartCurrentLimit(20);
    pivot.burnFlash();
  }

  @Override
  public void periodic() {
    // Check if we are at the setpoint
    if (isAtSetpoint()) {
      if (positionSetpoint == MotorSetPoint.PIVOT_LOAD) {
        // At load position, set motor output to zero to prevent pushing against the stop
        if (currentControlType != ControlType.kDutyCycle) {
          pid.setReference(0, ControlType.kDutyCycle);
          currentControlType = ControlType.kDutyCycle;
        }
      } else if (positionSetpoint == MotorSetPoint.PIVOT_SHOOT) {
        // At shoot position, continue holding position using position control
        if (currentControlType != ControlType.kPosition) {
          pid.setReference(positionSetpoint, ControlType.kPosition);
          currentControlType = ControlType.kPosition;
        }
      }
    } else {
      // Not at setpoint, ensure we are using the correct control type
      if (positionSetpoint == MotorSetPoint.PIVOT_LOAD || positionSetpoint == MotorSetPoint.PIVOT_SHOOT) {
        if (currentControlType != ControlType.kSmartMotion) {
          pid.setReference(positionSetpoint, ControlType.kSmartMotion);
          currentControlType = ControlType.kSmartMotion;
        }
      } else {
        // No setpoint specified
        pid.setReference(0, ControlType.kDutyCycle);
        currentControlType = ControlType.kDutyCycle;
      }
    }
  }

  // Pivot to loading position
  public void pivotLoadPosition() {
    positionSetpoint = MotorSetPoint.PIVOT_LOAD;
    pid.setSmartMotionMaxVelocity(MotorProperties.VORTEX_MAX_RPM, 0);
    pid.setSmartMotionMaxAccel(800, 0);
    pid.setReference(positionSetpoint, ControlType.kSmartMotion);
    currentControlType = ControlType.kSmartMotion;
  }

  // Pivot to shooting position
  public void pivotShootPosition() {
    if (noteTracker.hasNote()) {
      positionSetpoint = MotorSetPoint.PIVOT_SHOOT;
      pid.setSmartMotionMaxVelocity(MotorProperties.VORTEX_MAX_RPM, 0);
      pid.setSmartMotionMaxAccel(3000, 0);
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
