// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
  /** Creates a new IntakePivotSubsystem. */

  private final CANSparkMax PivotMotor;

  public CANSparkMax getPivotMotor() {
    return PivotMotor;
  }

  public IntakePivotSubsystem() {
    PivotMotor = new CANSparkMax(16, MotorType.kBrushless);
  }


// Run the Pivot Sub system in three modes
@Override
public void periodic(){
IntakePivotSubsystem.posGround();
}

private static void posGround() {
        throw new UnsupportedOperationException("Unimplemented method 'posGround'");
}



@Override
public void periodic(){
IntakePivotSubsystem.posScore();
}

private static void posScore() {
        throw new UnsupportedOperationException("Unimplemented method 'posScore'");
}






@Override
  public void periodic() {
    IntakePivotSubsystem.posDrive();
  }


private static void posDrive() {
    
    throw new UnsupportedOperationException("Unimplemented method 'posDrive'");
}


}





//FROM SATURDAY....

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake voltage", PivotMotor.getBusVoltage());
  }

