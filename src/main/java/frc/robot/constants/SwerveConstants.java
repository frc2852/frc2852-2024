// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {

  public static final class SwerveDrive {
    // Meters per second
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.74;

    // Radians per second
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    // Radians per second
    public static final double DIRECTION_SLEW_RATE = 1.2;

    // Percent per second (1 = 100%)
    public static final double MAGNITUDE_SLEW_RATE = 1.8;

    // Percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 2.0;

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(15.75);

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(15.75);

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSSIS_ANGULAR_OFFSET = -Math.PI / 2;
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
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static final class SwerveModule {

    // This should be configured based on the Swerve module type.
    // Turning motor is alwasy inverted in the SDS MK4i Swerve Module
    public static final boolean DRIVE_MOTOR_INVERTED = false;
    public static final boolean TURN_MOTOR_INVERTED = true;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the Swerve Module.
    public static final boolean TURN_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // TODO: Make this dynamic based on L1,L2,L3
    // Gear reduction ratio for the L2 stage of the SDS MK4i swerve module
    public static final double DRIVE_MOTOR_REDUCTION = 6.75;

    // TODO: Make this dynamic based on Drive motor configuration
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = VortexMotor.FREE_SPEED_RPM / 60;
    public static final double DRIVE_WHEEL_FREEE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVE_MOTOR_REDUCTION;

    public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_MOTOR_REDUCTION; // Meters
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0; // Meters per second

    // TODO: Reimplement this in SDSMK4i Swerve Module
    public static final double TURN_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Radians per second
    public static final double TURN_ENCODER_POSITION_PID_MIN_INPUT = 0; // Radians
    public static final double TURN_ENCODER_POSITION_PID_MAX_INPUT = TURN_ENCODER_POSITION_FACTOR; // Radians

    public static final double DRIVE_P = 0.04;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
    public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREEE_SPEED_RPS;

    public static final double TURN_P = 0.55;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0.01;
    public static final double TURN_FF = 0;

    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // amps
    public static final int TURN_MOTOR_CURRENT_LIMIT = 30; // amps
  }

  public static final class VortexMotor {
    public static final double FREE_SPEED_RPM = 6704;
  }

  public static final class NeoMotor {
    public static final double FREE_SPEED_RPM = 5676;
  }
}
