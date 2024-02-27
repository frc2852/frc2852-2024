// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 


public class conveyorSubsystem extends SubsystemBase {
 private CANSparkFlex topconveyor;
 private CANSparkFlex bottomconveyor;

 public DigitalInput conveyerlimitswitch;
 private double topconveyorshooter= 3;
 private double bottomconveyorshooter= 3;

public conveyorSubsystem (){
    topconveyor = new CANSparkFlex(14, MotorType.kBrushless);
    topconveyor = .setInverted(false);
    topconveyor = .setIdleMode( IdleMode.kCoast);
    topconveyor.burnFlash();

    bottomconveyor = new CANSparkFlex(15 , MotorType.kBrushless);
    bottomconveyor = .setInverted(true);
    bottomconveyor = .setIdleMode( IdleMode.kCoast);
    bottomconveyor.burnFlash();

    conveyerlimitswitch= new DigitalInput(4);
} 
private void conveyerMotorSpeed( double motorSpeedPercentage){
    double conveyerSpeed= motorSpeedPercentage/100;
    topconveyor.set(conveyerSpeed);
    bottomconveyor.set(conveyerSpeed);
}

public void stopConveyer (){
    topconveyor.set(0);
    bottomconveyor.set(0); 

     }
}


