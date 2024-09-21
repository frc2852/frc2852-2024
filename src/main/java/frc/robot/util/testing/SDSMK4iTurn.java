package frc.robot.util.testing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveConstants.SwerveModule;
import frc.robot.util.hardware.CANCoder;
import frc.robot.util.hardware.CANDevice;
import frc.robot.util.hardware.SparkMax;
import frc.robot.util.hardware.SparkMax.MotorModel;

public class SDSMK4iTurn {
  private final CANDevice turnDevice;
  private final SparkMax turnMotor;
  private final CANCoder turnEncoder;
  private final PIDController turnPIDController;

  public SDSMK4iTurn(CANDevice turnDevice, CANDevice encoderDevice, boolean invert) {
    this.turnDevice = turnDevice;

    // Initialize turning motor and encoder
    turnMotor = new SparkMax(turnDevice, MotorModel.NEO);
    turnMotor.setInverted(invert);
    turnMotor.setIdleMode(SwerveModule.TURN_MOTOR_IDLE_MODE);
    turnMotor.setSmartCurrentLimit(SwerveModule.TURN_MOTOR_CURRENT_LIMIT);

    turnEncoder = new CANCoder(encoderDevice, SwerveModule.TURN_ENCODER_INVERTED);
    turnPIDController = new PIDController(SwerveModule.TURN_P, SwerveModule.TURN_I, SwerveModule.TURN_D);

    // Set up the PID controller for continuous input for 0 to 2 * PI radians
    turnPIDController.enableContinuousInput(0, SwerveModule.TURN_ENCODER_POSITION_FACTOR);

    // Flash motor configurations to memory
    turnMotor.burnFlash();
  }

  // Set the desired angle (in radians) for the turn module
  public void setAngle(double angleRadians) {
    // Get the current angle in radians
    double currentAngleRadians = getAngle();

    SmartDashboard.putNumber(turnDevice.getDeviceName() + "CurrentAngle", Math.toDegrees(currentAngleRadians));

    // Calculate PID output to reach the target angle
    double turnOutput = turnPIDController.calculate(currentAngleRadians, angleRadians);

    // Set motor output based on PID output
    turnMotor.set(turnOutput);
  }

  // Get the current angle from the encoder in radians
  public double getAngle() {
    // The encoder gives a value between 0 and 1, representing full rotation, so multiply by 2 * PI
    return turnEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
  }

  // Update PID values
  public void updatePID(double p, double i, double d) {
    turnPIDController.setP(p);
    turnPIDController.setI(i);
    turnPIDController.setD(d);
  }
}
