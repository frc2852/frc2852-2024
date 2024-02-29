// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public ConvSubSystem {
private CANSparkFlex TopBelt;
private CANSparkFlex BottomBelt;
	
private double TopBeltMaxSpeed = 0.75;
private double BottomBeltMaxSpeed = 0.75;
}
  
TopBelt = new CANSparkFlex(deviceID:14, MotorType.kBrushless);
		TopMotor.setInverted(False);
		TopMotor.setIdleMode(IdleMode.kCoast);


BottomBelt = new CANSparkFlex(deviceID:15, MotorType.kBrushless);
		BottomMotor.setInverted(True);
		BottomMotor.setIdleMode(IdleMode.kCoast);


public class ConvSubSystem extends SubsystemBase {
  private Object m_drivercontroller;

  public ConvSubSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


private void configurebindings() {
m_drivercontroller.a().whiletrue(new RunCommand(ConvSubsystem,runConvIn());

}

public void runConvIn() {
		LoggingManager.log(String.format(
				"Running Conv: WheelSpeed=%.75f (Inverted: %b), RollerSpeed=%.75f (Inverted: %b)",
				wheelsConvMaxSpeed,
				BottomBelt.getInverted(),
				TopBelt.getInverted()),
				MessageType.DEBUG);


        public void periodic() {
        public void StopConv();
        }


		// Initialize the Contoller

    myXboxController = new XboxController(0);
      myXboxController.a.whenPressed(new RunCommand(ConvSubSystem,SpeakerPOS()());
      myXboxController.b.WhenPressed(new RunCommand(ConvSubsystem,AmpPOS()());
     


	  public void SpeakerPOS() {
		LoggingManager.log(String.format(
				"Running Speaker POS: WheelSpeed=%.5f (Inverted: %b)-(Inverted: %b), BeltSpeed=%.5f (Inverted: %b)",
				
				TopBelt.getInverted(),
				BottomBelt.getInverted(),
				topMotor.getInverted()),
				MessageType.DEBUG);


        public void periodic() {
        public void StopConv();


public void AmpPOS() {
		LoggingManager.log(String.format(
				"Running AMP POS: WheelSpeed=%.75f (Inverted: %b)-(Inverted: %b), BeltSpeed=%.75f (Inverted: %b)",
				
				TopBelt.getInverted(),
				BottomBelt.getInverted(),
				topMotor.getInverted()),
				MessageType.DEBUG);

   		public void periodic() {
        public void StopConv();


		
		@Override
		public void teleopPeriodic() {
		Object motor;
		motor.set(controller.getY())
		}




	public Object getM_drivercontroller() {
		return m_drivercontroller;
	}

	public void setM_drivercontroller(Object m_drivercontroller) {
		this.m_drivercontroller = m_drivercontroller;
	}