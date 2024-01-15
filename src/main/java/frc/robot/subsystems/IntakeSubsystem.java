package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 1) Intake Sub System
public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax TopMotor;
  private final CANSparkMax BottomMotor;

  // initialize the Bottom motor to run anti clockwise so that it runs along with
  // the Top motor which will run clockwise
  public IntakeSubsystem() {
    TopMotor = new CANSparkMax(14, MotorType.kBrushless);
    BottomMotor = new CANSparkMax(15, MotorType.kBrushless);
    BottomMotor.setInverted(true);
  }

// Run and stop the intake motors - 
@Override
public void periodic(){
IntakeSubsystem.IntakeGamePiece();
}

private static void IntakeGamePiece() {
        throw new UnsupportedOperationException("Unimplemented method 'IntakeGamePiece'");
}



@Override
public void periodic(){
IntakeSubsystem.OuttakeGamePiece();
}

private static void OuttakeGamePiece() {
        throw new UnsupportedOperationException("Unimplemented method 'OuttakeGamePiece'");
}



@Override
  public void periodic() {
    IntakeSubsystem.stopIntake();
  }


private static void stopIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stopIntake'");
}


//FROM SATURDAY....
@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake voltage", TopMotor.getBusVoltage());
  }

}
