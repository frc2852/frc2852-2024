// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkFlex topRoller;
  private final CANSparkFlex bottomRoller;

  private final DigitalInput intakeLimitSwitch;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    topRoller = new CANSparkFlex(10, MotorType.kBrushless);
    topRoller.setIdleMode(IdleMode.kCoast);
    topRoller.setInverted(false);
    topRoller.burnFlash();

    bottomRoller = new CANSparkFlex(11, MotorType.kBrushless);
    bottomRoller.setIdleMode(IdleMode.kCoast);
    bottomRoller.setInverted(true);
    bottomRoller.burnFlash();

    intakeLimitSwitch = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeIn() {
    topRoller.set(0.5);
    bottomRoller.set(0.5);
  }

  public void intakeReverse() {
    topRoller.set(-0.5);
    bottomRoller.set(-0.5);
  }

  public void intakeStop() {
    topRoller.set(0);
    bottomRoller.set(0);
  }

  public boolean isNoteInIntake() {
    return intakeLimitSwitch.get();
  }
}
