// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public ShooterSubsystem {
private CANSparkFlex TopRoller;
private CANSparkFlex BottomRoller;
	
private double TopRollerrMaxSpeed = 0.9;
private double BottomRollerMaxSpeed = 0.;
}
  
TopRoller = new CANSparkFlex(deviceID:12, MotorType.kBrushless);
		TopRoller.setInverted(False);
		TopRoller.setIdleMode(IdleMode.kCoast);


BottomRoller = new CANSparkFlex(deviceID:13, MotorType.kBrushless);
		BottomRoller.setInverted(True);
		BottomRoller.setIdleMode(IdleMode.kCoast);


public class ShooterSubsystem extends SubsystemBase {
  private Object m_drivercontroller;

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


private void configurebindings() {
m_drivercontroller.a().whiletrue(new RunCommand(ShooterSubsystem,ShootgamepieceSpeaker());
m_drivercontroller.b().whiletrue(new RunCommand(ShooterSubsystem,ShootgamepieceStage());
m_drivercontroller.c().whiletrue(new RunCommand(ShooterSubsystem,Divertgamepiece());
}



        public void ShootgamepieceSpeaker() {
		LoggingManager.log(String.format(
				"Running Shooter Speaker: WheelSpeed=%.5f (Inverted: %b)-(Inverted: %b), RollerSpeed=%.5f (Inverted: %b)",
				
				TopRoller.getInverted(),
				BottomRoller.getInverted(),
				topMotor.getInverted()),
				MessageType.DEBUG);


        public void periodic() {
        public void StopShooter();


public void ShootgamepieceStage() {
		LoggingManager.log(String.format(
				"Running Shooter Stage: WheelSpeed=%.9f (Inverted: %b)-(Inverted: %b), RollerSpeed=%.9f (Inverted: %b)",
				
				TopRoller.getInverted(),
				BottomRoller.getInverted(),
				topMotor.getInverted()),
				MessageType.DEBUG);

   		public void periodic() {
        public void StopShooter();


public void Divertgamepiece() {
		LoggingManager.log(String.format(
				"Running Shooter Divert game piece: WheelSpeed=%.2f (Inverted: %b)-(Inverted: %b), RollerSpeed=%.2f (Inverted: %b)",
				
				TopRoller.getInverted(),
				BottomRoller.getInverted(),
				topMotor.getInverted()),
				MessageType.DEBUG);

        public void periodic() {
        public void StopShooter();


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
    Object TopRoller;
	TopRoller.drive(0.25);  // Run at 25% speed
	}

	
	@Override
    public void execute() {
    Object BottomRoller;
	BottomRoller.drive(0.25);  // Run at 25% speed
	}

	public Object getM_drivercontroller() {
		return m_drivercontroller;
	}

	public void setM_drivercontroller(Object m_drivercontroller) {
		this.m_drivercontroller = m_drivercontroller;
	}