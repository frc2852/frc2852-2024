package frc.robot.util.testing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveConstants.SwerveModule;
import frc.robot.util.hardware.CANDevice;
import frc.robot.util.hardware.CANSpark;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SDSMK4iTurn {

  private final CANSpark turnMotor;
  private final CANcoder turnEncoder;
  private final PIDController turnPIDController;

  public SDSMK4iTurn(CANDevice turnDevice, CANDevice encoderDevice) {
    // Initialize turning motor and encoder
    turnMotor = new CANSpark(turnDevice);
    turnEncoder = new CANcoder(encoderDevice.getCanId());

    // Set up the PID controller for radians (0 - 2 * PI)
    turnPIDController = new PIDController(0.55, 0, 0.01);
    // Continuous input for 0 to 2 * PI radians
    turnPIDController.enableContinuousInput(0, 2 * Math.PI);

    // CANCoder configuration
    CANcoderConfiguration config = new CANcoderConfiguration();
    CANcoderConfigurator canCoderConfigurator = turnEncoder.getConfigurator();
    CANcoderConfiguration oldConfig = new CANcoderConfiguration();
    canCoderConfigurator.refresh(oldConfig);
    config.MagnetSensor.MagnetOffset = oldConfig.MagnetSensor.MagnetOffset;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; 
    turnEncoder.getConfigurator().apply(config);

    // Set motor limits and modes
    turnMotor.setIdleMode(SwerveModule.TURNING_MOTOR_IDLE_MODE);
    turnMotor.setSmartCurrentLimit(SwerveModule.TURNING_MOTOR_CURRENT_LIMIT);

    // Flash motor configurations to memory
    turnMotor.burnFlash();
  }

  // Set the desired angle (in radians) for the turn module
  public void setAngle(double angleRadians) {
    // Get the current angle in radians
    double currentAngleRadians = getAngle();

    SmartDashboard.putNumber(turnMotor.GetDeviceName() + "CurrentAngle", Math.toDegrees(currentAngleRadians));

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
