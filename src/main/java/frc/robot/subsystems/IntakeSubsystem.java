// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public IntakeSubsystem {
private CANSparkFlex TopMotor;
private CANSparkFlex BottomMotor;
	
private double wheelsIntakeMaxSpeed = 0.6;
private double rollerIntakeMaxSpeed = 0.6;
}
  
TopMotor = new CANSparkFlex(deviceID:10, MotorType.kBrushless);
		TopMotor.setInverted(False);
		TopMotor.setIdleMode(IdleMode.kCoast);


BottomMotor = new CANSparkFlex(deviceID:11, MotorType.kBrushless);
		BottomMotor.setInverted(True);
		BottomMotor.setIdleMode(IdleMode.kCoast);


public class IntakeSubsystem extends SubsystemBase {
  private Object m_drivercontroller;

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


private void configurebindings() {
m_drivercontroller.a().whiletrue(new RunCommand(IntakeSubsystem,runIntakeIn());

}

public void runIntakeIn() {
		LoggingManager.log(String.format(
				"Running intake: WheelSpeed=%.2f (Inverted: %b), RollerSpeed=%.2f (Inverted: %b)",
				wheelsIntakeMaxSpeed,
				BottomMotor.getInverted(),
				rollerIntakeMaxSpeed,
				topMotor.getInverted()),
				MessageType.DEBUG);


        public void runIntakeOut() {
		LoggingManager.log(String.format(
				"Running intake out: WheelSpeed=%.2f (Inverted: %b)-(Inverted: %b), RollerSpeed=%.2f (Inverted: %b)",
				wheelsIntakeMaxSpeed,
				leftMotor.getInverted(),
				rightMotor.getInverted(),
				rollerIntakeMaxSpeed,
				topMotor.getInverted()),
				MessageType.DEBUG);


        public void periodic() {
        public void StopIntake();
        }


		// Initialize the Contoller

    myXboxController = new XboxController(0);
      myXboxController.a.whenPressed(new MoveCommand());
      myXboxController.b.WhenPressed(new MoveCommand());
      myXboxController.x.whenPressed(new MoveCommand());
 	  myXboxController.y.whenPressed(new MoveCommand());

		//Run both motors at 25% speed when Y button is pressed on the Controller 

		@Override
		public void teleopPeriodic() {
		Object motor;
		motor.set(controller.getY())
		}


		@Override
    public void execute() {
    Object TopMotor;
	TopMotor.drive(0.25);  // Run at 25% speed
	}

	
	@Override
    public void execute() {
    Object BottomMotor;
	BottomMotor.drive(0.25);  // Run at 25% speed
	}

	public Object getM_drivercontroller() {
		return m_drivercontroller;
	}

	public void setM_drivercontroller(Object m_drivercontroller) {
		this.m_drivercontroller = m_drivercontroller;
	}