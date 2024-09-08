// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.SwerveConstants.SwerveModule;
import frc.robot.util.hardware.CANDevice;
import frc.robot.util.hardware.CANSpark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;

public class SDSMK4iSwerveModule {

  private final CANSpark driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkPIDController drivePIDController;

  private final CANSpark turnMotor;
  private final CANcoder turnEncoder;
  private final PIDController turnPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SDSMK4iSwerveModule(CANDevice driveCANDevice, Boolean driveInverted, CANDevice turnCANDevice, CANDevice encoderCANDevice, double chassisAngularOffset) {
    driveMotor = new CANSpark(driveCANDevice);
    drivePIDController = driveMotor.getPIDController();
    driveEncoder = driveMotor.getEncoder();
    drivePIDController.setFeedbackDevice(driveEncoder);
    driveMotor.setInverted(driveInverted);
    driveMotor.setIdleMode(SwerveModule.DRVING_MOTOR_IDLE_MODE);
    driveMotor.setSmartCurrentLimit(SwerveModule.DRIVING_MOTOR_CURRENT_LIMIT);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(SwerveModule.DRIVING_ENCODER_POSITION_FACTOR);
    driveEncoder.setVelocityConversionFactor(SwerveModule.DRIVING_ENCODER_VELOCITY_FACTOR);

    drivePIDController.setP(SwerveModule.DRIVING_P);
    drivePIDController.setI(SwerveModule.DRIVING_I);
    drivePIDController.setD(SwerveModule.DRIVING_D);
    drivePIDController.setFF(SwerveModule.DRIVING_FF);
    drivePIDController.setOutputRange(SwerveModule.DRIVING_MIN_OUTPUT, SwerveModule.DRIVING_MAX_OUTPUT);

    driveMotor.burnFlash();

    // Turning motor configuration
    turnMotor = new CANSpark(turnCANDevice);
    turnMotor.setInverted(SwerveModule.TURN_INVERTED);
    turnMotor.setIdleMode(SwerveModule.TURNING_MOTOR_IDLE_MODE);
    turnMotor.setSmartCurrentLimit(SwerveModule.TURNING_MOTOR_CURRENT_LIMIT);

    turnPIDController = new PIDController(0.55, 0, 0.01);
    turnPIDController.enableContinuousInput(0, 2 * Math.PI);

    turnEncoder = new CANcoder(encoderCANDevice.getCanId());
    CANcoderConfiguration config = new CANcoderConfiguration();
    CANcoderConfigurator canCoderConfigurator = turnEncoder.getConfigurator();
    CANcoderConfiguration oldConfig = new CANcoderConfiguration();
    canCoderConfigurator.refresh(oldConfig);
    config.MagnetSensor.MagnetOffset = oldConfig.MagnetSensor.MagnetOffset;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    turnEncoder.getConfigurator().apply(config);
    turnMotor.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(getAngle());
    driveEncoder.setPosition(0);
  }

    /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(getAngle() - chassisAngularOffset));
  }

    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(getAngle() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(getAngle()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnMotor.set(turnPIDController.calculate(getAngle(), optimizedDesiredState.angle.getRadians()));

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public double getAngle() {
    // The encoder gives a value between 0 and 1, representing full rotation, so multiply by 2 * PI
    return turnEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
  }
}