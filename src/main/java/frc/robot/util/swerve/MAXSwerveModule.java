// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.SwerveConstants.SwerveModule;
import frc.robot.util.hardware.CANDevice;
import frc.robot.util.hardware.SparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class MAXSwerveModule {
  private final SparkMax driveSpark;
  private final SparkMax turnSpark;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(CANDevice driveCANDevice, CANDevice turnCANDevice, double chassisAngularOffset) {
    driveSpark = new SparkMax(driveCANDevice, SparkMax.MotorModel.NEO);
    turnSpark = new SparkMax(turnCANDevice, SparkMax.MotorModel.NEO);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = driveSpark.getEncoder();
    turningEncoder = turnSpark.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = driveSpark.getPIDController();
    turningPIDController = turnSpark.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(SwerveModule.DRIVE_ENCODER_POSITION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(SwerveModule.DRIVE_ENCODER_VELOCITY_FACTOR); 

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(SwerveModule.TURN_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(SwerveModule.TURN_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    driveSpark.setInverted(false);
    turningEncoder.setInverted(SwerveModule.TURN_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(SwerveModule.TURN_ENCODER_POSITION_PID_MIN_INPUT);
    turningPIDController.setPositionPIDWrappingMaxInput(SwerveModule.TURN_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingPIDController.setP(SwerveModule.DRIVE_P);
    drivingPIDController.setI(SwerveModule.DRIVE_I);
    drivingPIDController.setD(SwerveModule.DRIVE_D);
    drivingPIDController.setFF(SwerveModule.DRIVE_FF);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPIDController.setP(SwerveModule.TURN_P);
    turningPIDController.setI(SwerveModule.TURN_I);
    turningPIDController.setD(SwerveModule.TURN_D);
    turningPIDController.setFF(SwerveModule.TURN_FF);

    driveSpark.setIdleMode(SwerveModule.DRIVE_MOTOR_IDLE_MODE);
    turnSpark.setIdleMode(SwerveModule.TURN_MOTOR_IDLE_MODE);
    driveSpark.setSmartCurrentLimit(SwerveModule.DRIVE_MOTOR_CURRENT_LIMIT);
    turnSpark.setSmartCurrentLimit(SwerveModule.TURN_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveSpark.burnFlash();
    turnSpark.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
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
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
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
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }
}