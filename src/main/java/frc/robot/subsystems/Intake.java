package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.DIOId;
import frc.robot.constants.Constants.MotorProperties;
import frc.robot.constants.Constants.MotorSetPoint;
import frc.robot.util.NoteTracker;
import frc.robot.util.hardware.SparkFlex;

public class Intake extends SubsystemBase {

  // Controllers
  private final SparkFlex topRollers = new SparkFlex(CANBus.INTAKE_TOP);
  private final SparkFlex bottomRollers = new SparkFlex(CANBus.INTAKE_LOWER);

  // Sensors
  private final DigitalInput intakeBeamBreak;
  private final DigitalInput shooterBeamBreak;

  // State
  private final NoteTracker noteTracker;
  private double velocitySetpoint;

  // State Machine
  private enum IntakeState {
    SEEKING, // No note, running intake at half speed
    ACQUIRING, // Note detected, running intake at full speed
    HOLDING, // Note held in intake, waiting to be shot
    DELIVER
  }

  private IntakeState currentState = IntakeState.SEEKING;

  public Intake(NoteTracker noteTracker) {
    this.noteTracker = noteTracker;

    // Set motor controller configurations
    topRollers.setIdleMode(IdleMode.kBrake);
    topRollers.setInverted(true);
    topRollers.pidParameters.SetPID(0.0001, 0.000001, 0);
    topRollers.burnFlash();

    bottomRollers.setIdleMode(IdleMode.kBrake);
    bottomRollers.setInverted(true);
    bottomRollers.pidParameters.SetPID(0.0001, 0.000001, 0);
    bottomRollers.burnFlash();

    // Initialize sensors
    intakeBeamBreak = new DigitalInput(DIOId.INTAKE_BEAM_BREAK);
    shooterBeamBreak = new DigitalInput(DIOId.SHOOTER_BEAM_BREAK);
  }

  @Override
  public void periodic() {
    // Reset to SEEKING if note has been shot
    if (!noteTracker.hasNote() && currentState != IntakeState.SEEKING) {
      currentState = IntakeState.SEEKING;
    }

    switch (currentState) {
      case SEEKING:
        if (!noteTracker.hasNote() && !isGamePieceDetected()) {
          // No note, no detection
          runIntakeHalfSpeed();
        } else if (isGamePieceDetected()) {
          // Detected a game piece
          noteTracker.setNoteAcquired();
          currentState = IntakeState.ACQUIRING;
          runIntakeFullSpeed();
        }
        break;

      case ACQUIRING:
        if (isNoteAtShooter()) {
          currentState = IntakeState.HOLDING;
        } else {
          // Keep running intake at full speed
          runIntakeFullSpeed();
        }
        break;
      case HOLDING:
        // Note is held; intake is stopped
        stopIntake();
        break;
      case DELIVER:
        // Note is held; move note to shooter
        runIntakeFullSpeedForReal();
        break;
    }
  }

  public void deliverNoteToShooter() {
    currentState = IntakeState.DELIVER;
  }

  public boolean isNoteAtShooter() {
    return !shooterBeamBreak.get();
  }

  private void runIntakeHalfSpeed() {
    velocitySetpoint = MotorSetPoint.INTAKE_HALF;
    topRollers.stopMotor();
    bottomRollers.setVelocity(velocitySetpoint);
  }

  private void runIntakeFullSpeed() {
    velocitySetpoint = MotorSetPoint.INTAKE_FULL;
    topRollers.setVelocity(velocitySetpoint);
    bottomRollers.setVelocity(velocitySetpoint);
  }

  private void runIntakeFullSpeedForReal() {
    velocitySetpoint = MotorProperties.VORTEX_MAX_RPM;
    topRollers.setVelocity(velocitySetpoint);
    bottomRollers.setVelocity(velocitySetpoint);
  }

  private void stopIntake() {
    velocitySetpoint = MotorSetPoint.STOP;
    topRollers.stopMotor();
    bottomRollers.stopMotor();
  }

  private boolean isGamePieceDetected() {
    return !intakeBeamBreak.get();
  }
}
