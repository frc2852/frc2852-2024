package frc.robot.util.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveConstants.SwerveModule;
import frc.robot.util.SparkMax;
import frc.robot.util.SparkMax.MotorModel;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SDSMK4iTurnModule {

  private final SparkMax turnMotor;
  private final CANcoder turnEncoder;
  private final PIDController turnPIDController;

  public SDSMK4iTurnModule(int turningCANId, int canCoderCANId) {
    // Initialize turning motor and encoder
    turnMotor = new SparkMax(turningCANId, MotorModel.NEO);
    turnEncoder = new CANcoder(canCoderCANId);

    // Set up the PID controller for degrees (0 - 360)
    turnPIDController = new PIDController(0.01, 0, 0);
    // Continuous input for 0 to 360 degrees
    turnPIDController.enableContinuousInput(0, 360);

    // CANCoder configuration
    CANcoderConfiguration config = new CANcoderConfiguration();
    CANcoderConfigurator canCoderConfigurator = turnEncoder.getConfigurator();
    CANcoderConfiguration oldConfig = new CANcoderConfiguration();
    canCoderConfigurator.refresh(oldConfig);
    config.MagnetSensor.MagnetOffset = oldConfig.MagnetSensor.MagnetOffset;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    turnEncoder.getConfigurator().apply(config);

    // Set motor limits and modes
    turnMotor.setIdleMode(SwerveModule.TURNING_MOTOR_IDLE_MODE);
    turnMotor.setSmartCurrentLimit(SwerveModule.TURNING_MOTOR_CURRENT_LIMIT);

    // Flash motor configurations to memory
    turnMotor.burnFlash();
  }

  // Set the desired angle (in degrees) for the turn module
  public void setAngle(double angleDegrees) {
    // Get the current angle in degrees
    double currentAngleDegrees = getAngle();

    // Calculate PID output to reach the target angle
    double turnOutput = turnPIDController.calculate(currentAngleDegrees, angleDegrees);

    // Set motor output based on PID output
    turnMotor.set(turnOutput);
  }

  // Get the current angle from the encoder in degrees
  public double getAngle() {
    // The encoder gives a value between 0 and 1, representing full rotation, so multiply by 360
    return turnEncoder.getAbsolutePosition().getValue() * 360;
  }
}
