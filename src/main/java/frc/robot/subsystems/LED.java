package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.NoteTracker;

public class LED extends SubsystemBase {

  // The PWM port where the Blinkin is connected
  private final PWM blinkin;

  // Enum to represent supported colors and their corresponding PWM values
  public enum LEDColor {
    RED(0.61),
    GREEN(0.77),
    BLUE(0.87),
    YELLOW(0.69),
    WHITE(0.93),
    ORANGE(0.65),
    PURPLE(0.91),
    OFF(0.99); // OFF case is mapped to 0.99 (no output)

    private final double pwmValue;

    // Constructor to assign the PWM value to each color
    LEDColor(double pwmValue) {
      this.pwmValue = pwmValue;
    }

    // Getter for PWM value
    public double getPWMValue() {
      return this.pwmValue;
    }
  }

  // State
  private final NoteTracker noteTracker;

  // Constructor to initialize the Blinkin on the given PWM port
  public LED(NoteTracker noteTracker) {
    this.noteTracker = noteTracker;
    blinkin = new PWM(Constants.PWM.BLINKIN);
  }

  @Override
  public void periodic() {
    if(noteTracker.hasNote()){
      setLEDColor(LEDColor.GREEN);
    } else {
      setLEDColor(LEDColor.OFF);
    }
  }

  /**
   * Sets the LED color using the predefined enum values.
   * 
   * @param color The LEDColor enum value representing the desired color.
   */
  private void setLEDColor(LEDColor color) {
    blinkin.setSpeed(color.getPWMValue());
  }
}
