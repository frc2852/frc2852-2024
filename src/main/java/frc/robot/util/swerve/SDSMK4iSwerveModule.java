package frc.robot.util.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import frc.robot.constants.Constants.MotorModel;
import frc.robot.constants.SwerveConstants.SwerveDrive;
import frc.robot.constants.SwerveConstants.SwerveModule;
import frc.robot.util.hardware.CANCoder;
import frc.robot.util.hardware.CANDevice;
import frc.robot.util.hardware.SparkFlex;
import frc.robot.util.hardware.SparkMax;

/**
 * Represents a single swerve module using the SDS MK4i configuration.
 * This class encapsulates the functionality of both the driving and turning motors,
 * their encoders, and PID controllers. It provides methods to get the current state
 * and position of the module, as well as to set the desired state.
 */
public class SDSMK4iSwerveModule {

    private final SparkFlex driveMotor;

    private final SparkMax turnMotor;
    private final CANcoder turnEncoder;
    private final PIDController turnPIDController;

    private final double chassisAngularOffset;
    private final Rotation2d chassisAngularOffsetRotation2d;

    @SuppressWarnings("unused")
    private SwerveModuleState desiredState;

    // Reusable objects to avoid repeated allocations
    private final SwerveModuleState correctedDesiredState = new SwerveModuleState(0.0, new Rotation2d());
    private final SwerveModuleState optimizedDesiredState = new SwerveModuleState(0.0, new Rotation2d());
    private Rotation2d currentRotation = new Rotation2d();

    /**
     * Constructs an SDSMK4iSwerveModule and configures the driving and turning motors,
     * encoders, and PID controllers.
     *
     * @param driveCANDevice       The CAN device for the drive motor.
     * @param turnCANDevice        The CAN device for the turning motor.
     * @param encoderCANDevice     The CAN device for the turning encoder.
     * @param chassisAngularOffset The angular offset of the module relative to the chassis.
     */
    public SDSMK4iSwerveModule(
            CANDevice driveCANDevice,
            CANDevice turnCANDevice,
            CANDevice encoderCANDevice,
            double chassisAngularOffset) {
        this.chassisAngularOffset = chassisAngularOffset;
        this.chassisAngularOffsetRotation2d = Rotation2d.fromRadians(chassisAngularOffset);

        // Initialize drive motor, encoder, and PID controller
        driveMotor = new SparkFlex(driveCANDevice);

        configureDriveMotor();

        // Initialize turning motor, encoder, and PID controller
        turnMotor = new SparkMax(turnCANDevice, MotorModel.NEO);
        turnEncoder = new CANCoder(encoderCANDevice, SwerveModule.TURN_ENCODER_INVERTED);
        turnPIDController = new PIDController(
                SwerveModule.TURN_P,
                SwerveModule.TURN_I,
                SwerveModule.TURN_D);

        configureTurnMotor();

        // Initialize desired state
        desiredState = new SwerveModuleState(0.0, new Rotation2d(getAbsolutePosition()));

        // Reset drive encoder position
        driveMotor.encoder.setPosition(0);
    }

    /**
     * Configures the drive motor settings.
     */
    private void configureDriveMotor() {
        driveMotor.pidController.setFeedbackDevice(driveMotor.encoder);
        driveMotor.setInverted(SwerveModule.DRIVE_MOTOR_INVERTED);
        driveMotor.setIdleMode(SwerveModule.DRIVE_MOTOR_IDLE_MODE);
        driveMotor.setSmartCurrentLimit(SwerveModule.DRIVE_MOTOR_CURRENT_LIMIT);

        // Apply position and velocity conversion factors for the driving encoder
        driveMotor.encoder.setPositionConversionFactor(SwerveModule.DRIVE_ENCODER_POSITION_FACTOR);
        driveMotor.encoder.setVelocityConversionFactor(SwerveModule.DRIVE_ENCODER_VELOCITY_FACTOR);

        // Set PID coefficients for the driving motor
        driveMotor.pidController.setP(SwerveModule.DRIVE_P);
        driveMotor.pidController.setI(SwerveModule.DRIVE_I);
        driveMotor.pidController.setD(SwerveModule.DRIVE_D);
        driveMotor.pidController.setFF(SwerveModule.DRIVE_FF);

        driveMotor.burnFlash();
    }

    /**
     * Configures the turning motor settings.
     */
    private void configureTurnMotor() {
        turnMotor.setInverted(SwerveModule.TURN_MOTOR_INVERTED);
        turnMotor.setIdleMode(SwerveModule.TURN_MOTOR_IDLE_MODE);
        turnMotor.setSmartCurrentLimit(SwerveModule.TURN_MOTOR_CURRENT_LIMIT);

        // Set up the PID controller for continuous input between 0 and 2 * PI radians
        turnPIDController.enableContinuousInput(0, SwerveDrive.TWO_PI);

        turnMotor.burnFlash();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double absolutePosition = getAbsolutePosition();
        return new SwerveModuleState(
                driveMotor.encoder.getVelocity(),
                getWheelAngle(absolutePosition));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double absolutePosition = getAbsolutePosition();
        return new SwerveModulePosition(
                driveMotor.encoder.getPosition(),
                getWheelAngle(absolutePosition));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired angle
        Rotation2d correctedAngle = desiredState.angle.plus(chassisAngularOffsetRotation2d);

        // Update the corrected desired state
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = correctedAngle;

        // Get current position once
        double absolutePosition = getAbsolutePosition();
        currentRotation = new Rotation2d(absolutePosition);

        // Optimize the reference state to avoid unnecessary rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
                correctedDesiredState,
                currentRotation);

        // Command the driving and turning motors to their respective setpoints
        driveMotor.pidController.setReference(
                optimizedState.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);

        double desiredAngleRadians = optimizedState.angle.getRadians();
        double turnOutput = turnPIDController.calculate(
                absolutePosition,
                desiredAngleRadians);
        turnMotor.set(turnOutput);

        this.desiredState = desiredState;
    }

    /**
     * Resets the drive encoder to zero.
     */
    public void resetEncoders() {
        driveMotor.encoder.setPosition(0);
    }

    /**
     * Gets the absolute position of the turning encoder in radians.
     *
     * @return The absolute position in radians.
     */
    private double getAbsolutePosition() {
        // The encoder provides a value between 0 and 1, representing a full rotation
        return turnEncoder.getAbsolutePosition().getValue() * SwerveDrive.TWO_PI;
    }

    /**
     * Calculates the wheel angle, accounting for the chassis angular offset.
     *
     * @return The wheel angle as a Rotation2d object.
     */
    private Rotation2d getWheelAngle(double absolutePosition) {
        double angle = absolutePosition - chassisAngularOffset;
        return new Rotation2d(angle);
    }
}
