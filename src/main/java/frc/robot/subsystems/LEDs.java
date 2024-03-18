// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.vision.Color;

public class LEDs extends SubsystemBase {

  private SerialPort led;

  public LEDs() {
    try {
      // Initialize the serial port for communication with the Arduino
      led = new SerialPort(9600, SerialPort.Port.kOnboard); // Adjust the baud rate as needed
    } catch (Exception e) {
      DriverStation.reportError("Error initializing LED serial port: " + e.getMessage(), true);
    }
  }

  /**
   * Sends a predefined color to the Arduino controlling the LEDs.
   * 
   * @param color The color to set the LEDs to.
   */
  public void setLEDColor(Color color) {
    if (led != null) {
      String colorString = String.format("%d,%d,%d", color.getRed(), color.getGreen(), color.getBlue());
      led.writeString(colorString + "\n");
    }
  }
}
