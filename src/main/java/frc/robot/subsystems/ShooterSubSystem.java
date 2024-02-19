// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubSystem extends SubsystemBase {

  private CANSparkFlex topShooter;
  private CANSparkFlex bottomShooter;
  private DigitalInput intakeLimitSwitch;
  private double wheelShooterMaxSpeed= 2;
  private double rolllShooterMaxSpeed= 2;
  private double stopIntake(0);// still don't know why is that a problem 

  public ShooterSubSystem() {
    topShooter= new CANSparkFlex(12, MotorType.kBrushless);
    topShooter.setInverted(false);
    topShooter.setIdleMode(IdleMode.kCoast);
    topShooter.burnFlash();

    bottomShooter = new CANSparkFlex(13, MotorType.kBrushless);
    bottomShooter.setInverted(true);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.burnFlash();

    intakeLimitSwitch = new DigitalInput(3);

  }
  @Override
  public void periodic() {}

  //setting speed for the shooter 
  private void shooterSpeed( double motorSpeedPercentage){
    double motorSpeed= motorSpeedPercentage/100;
    //copied this code in the last program
    topShooter.set(motorSpeed);
    bottomShooter.set(motorSpeed);
  }


  // shooter when the note is in, it stop then proceed to shoot
public void runShooter(){
  if (noteComplete()==true){
    stopIntake();// need to know if do i stop the shooter or intake if the note is in the shooter
  }else {
    topShooter.set(wheelShooterMaxSpeed);
    bottomShooter.set(rolllShooterMaxSpeed);
  }
}
private boolean noteComplete(){
  // != invert our value /or not equal to 
  boolean val= intakeLimitSwitch.get();//false
  boolean returnVal= !val;
  return!intakeLimitSwitch.get();
}
}
  

