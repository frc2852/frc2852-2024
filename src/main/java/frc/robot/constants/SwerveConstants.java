// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Constants used for configuring the swerve drive system.
 */
public final class SwerveConstants {

    public static final class SwerveDrive {
        // Maximum speed in meters per second
        public static final double MAX_SPEED_METERS_PER_SECOND = 5.74;

        // Maximum angular speed in radians per second
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

        // Direction slew rate in radians per second
        public static final double DIRECTION_SLEW_RATE = 1.2;

        // Magnitude slew rate as a percentage per second (1 = 100%)
        public static final double MAGNITUDE_SLEW_RATE = 1.8;

        // Rotational slew rate as a percentage per second (1 = 100%)
        public static final double ROTATIONAL_SLEW_RATE = 2.0;

        // Chassis configuration
        public static final double TRACK_WIDTH = Units.inchesToMeters(15.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(15.75);

        // Kinematics for the swerve drive
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),    // Front left
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // Front right
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   // Back left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)   // Back right
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

        public static final boolean GYRO_REVERSED = true;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED
            );
    }

    public static final class SwerveModule {
        // Motor inversion settings
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean TURN_MOTOR_INVERTED = true;

        // Invert the turning encoder since the output shaft rotates opposite to the steering motor
        public static final boolean TURN_ENCODER_INVERTED = true;

        // Wheel dimensions
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        // Gear reduction ratio for the SDS MK4i swerve module (L2 configuration)
        // TODO: Make this dynamic based on L1, L2, L3 configurations
        public static final double DRIVE_MOTOR_REDUCTION = 6.75;

        // Free speed of the drive motor in rotations per second
        // TODO: Make this dynamic based on drive motor configuration
        public static final double DRIVE_MOTOR_FREE_SPEED_RPS = VortexMotor.FREE_SPEED_RPM / 60.0;

        // Free speed of the wheel in meters per second
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = 
            (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVE_MOTOR_REDUCTION;

        // Encoder conversion factors
        public static final double DRIVE_ENCODER_POSITION_FACTOR = 
            WHEEL_CIRCUMFERENCE_METERS / DRIVE_MOTOR_REDUCTION; // Meters per rotation
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR = 
            DRIVE_ENCODER_POSITION_FACTOR / 60.0; // Meters per second

        // TODO: Reimplement this in SDS MK4i Swerve Module
        public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Radians per rotation
        public static final double TURN_ENCODER_VELOCITY_FACTOR = TURN_ENCODER_POSITION_FACTOR / 60.0; // Radians per second

        // PID controller input range
        public static final double TURN_ENCODER_POSITION_PID_MIN_INPUT = 0; // Radians
        public static final double TURN_ENCODER_POSITION_PID_MAX_INPUT = TURN_ENCODER_POSITION_FACTOR; // Radians

        // Drive motor PID constants
        public static final double DRIVE_P = 0.04;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;

        // Turn motor PID constants
        public static final double TURN_P = 0.620000;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0.000001;
        public static final double TURN_FF = 0;

        // Motor idle modes
        public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;

        // Motor current limits
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // Amps
        public static final int TURN_MOTOR_CURRENT_LIMIT = 40; // Amps
    }

    public static final class VortexMotor {
        public static final double FREE_SPEED_RPM = 6704;
    }

    public static final class NeoMotor {
        public static final double FREE_SPEED_RPM = 5676;
    }
}
