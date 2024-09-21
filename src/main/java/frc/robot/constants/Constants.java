// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.util.hardware.CANDevice;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstant {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int SYSID_CONTROLLER_PORT = 2;

    public static final double DEAD_BAND = 0.15;
    public static final double EXPONENTIAL_RESPONSE = 3;
  }

  public static class CANBus {

    public static final CANDevice FRONT_LEFT_DRIVE = new CANDevice("Drive", "FrontLeftDrive", 2);
    public static final CANDevice FRONT_LEFT_TURN = new CANDevice("Drive", "FrontLeftTurn", 3);
    public static final CANDevice FRONT_LEFT_ENCODER = new CANDevice("Drive", "FrontLeftEncoder", 4);

    public static final CANDevice FRONT_RIGHT_DRIVE = new CANDevice("Drive", "FrontRightDrive", 5);
    public static final CANDevice FRONT_RIGHT_TURN = new CANDevice("Drive", "FrontRightTurn", 6);
    public static final CANDevice FRONT_RIGHT_ENCODER = new CANDevice("Drive", "FrontRightEncoder", 7);

    public static final CANDevice REAR_LEFT_DRIVE = new CANDevice("Drive", "RearLeftDrive", 8);
    public static final CANDevice REAR_LEFT_TURN = new CANDevice("Drive", "RearLeftTurn", 9);
    public static final CANDevice REAR_LEFT_ENCODER = new CANDevice("Drive", "RearLeftEncoder", 10);

    public static final CANDevice REAR_RIGHT_DRIVE = new CANDevice("Drive", "RearRightDrive", 11);
    public static final CANDevice REAR_RIGHT_TURN = new CANDevice("Drive", "RearRightTurn", 12);
    public static final CANDevice REAR_RIGHT_ENCODER = new CANDevice("Drive", "RearRightEncoder", 13);

    public static final CANDevice INTAKE_LOWER = new CANDevice("Intake", "LowerRoller", 14);
    public static final CANDevice INTAKE_TOP = new CANDevice("Intake", "TopRoller", 15);
    
    public static final CANDevice SHOOTER_PIVOT = new CANDevice("Shooter", "Pivot", 16);
    public static final CANDevice SHOOTER_LEFT = new CANDevice("Shooter", "Left", 17);
    public static final CANDevice SHOOTER_RIGHT = new CANDevice("Shooter", "Right", 18);
  }

  public static class DIOId {
    public static final int SHOOTER_BEAM_BREAK = 0;
    public static final int INTAKE_BEAM_BREAK = 1;
  }

  public static class MotorSetPoint {
    public static final int STOP = 0;
    public static final int INTAKE_HALF = 2000;
    public static final int INTAKE_FULL = 5000;
  }
}
