package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.Constants.DIOId;
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
    if (!isGamePieceDeteted() && !isGamePieceLoaded()) {
      runIntakeHalfSpeed();
    } else if (isGamePieceDeteted()) {
      noteTracker.setNoteAcquired();
      runIntakeFullSpeed();
    } else if (isGamePieceLoaded()) {
      stopIntake();
      moveNoteAwayFromShooter();
    }

    SmartDashboard.putBoolean("isGamePieceDeteted", isGamePieceDeteted());
    SmartDashboard.putBoolean("isGamePieceLoaded", isGamePieceLoaded());
    SmartDashboard.putBoolean("isNoteAtShooter", isNoteAtShooter());

    topRollers.periodic();
    bottomRollers.periodic();
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

  private void stopIntake() {
    velocitySetpoint = MotorSetPoint.STOP;
    topRollers.stopMotor();
    bottomRollers.stopMotor();
  }

  private void moveNoteAwayFromShooter() {
    var currentPosition = topRollers.encoder.getPosition();
    var targetPosition = currentPosition - MotorSetPoint.INTAKE_REVERSE_POSITION;
    topRollers.setPosition(targetPosition);
  }

  private boolean isGamePieceDeteted() {
    return !intakeBeamBreak.get();
  }

  private boolean isGamePieceLoaded() {
    return isNoteAtShooter() || noteTracker.hasNote();
  }

  private boolean isNoteAtShooter() {
    return !shooterBeamBreak.get();
  }
}