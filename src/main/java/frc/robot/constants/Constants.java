// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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

  public static class CanbusId {
    public static final int POWER_DISTRIBUTION_HUB = 1;
    public static final int FRONT_LEFT_DRIVE = 2, FRONT_LEFT_TURNING = 3, FRONT_LEFT_ENCODER = 4;
    public static final int REAR_LEFT_DRIVE = 5, REAR_LEFT_TURNING = 6, REAR_LEFT_ENCODER = 7;
    public static final int FRONT_RIGHT_DRIVE = 8, FRONT_RIGHT_TURNING = 9, FRONT_RIGHT_ENCODER = 10;
    public static final int REAR_RIGHT_DRIVE = 11, REAR_RIGHT_TURNING = 12, REAR_RIGHT_ENCODER = 13;
  }
}
