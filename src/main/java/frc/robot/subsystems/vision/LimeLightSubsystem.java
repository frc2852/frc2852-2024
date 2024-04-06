// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimeLightSubsystem extends SubsystemBase {

  // Network Table for LimeLight
  private final NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // LimeLight data entries
  private final NetworkTableEntry tv = limeLightTable.getEntry("tv");
  private final NetworkTableEntry tx = limeLightTable.getEntry("tx");
  private final NetworkTableEntry ty = limeLightTable.getEntry("ty");

  // Constants
  private static final double TARGET_HEIGHT = 0.0254; // Height of the target in meters
  private static final double LIMELIGHT_HEIGHT = 0.4191; // Height of the LimeLight from the ground in meters
  private static final double LIMELIGHT_MOUNT_ANGLE = 43.0; // Angle at which the LimeLight is mounted

  public LimeLightSubsystem() {
    // Set default command if needed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update SmartDashboard with LimeLight values
    SmartDashboard.putBoolean("Target Visible", isTargetVisible());
    SmartDashboard.putNumber("Horizontal Offset", getHorizontalOffset());
    SmartDashboard.putNumber("Vertical Offset", getVerticalOffset());
    SmartDashboard.putNumber("Distance to Target", calculateDistanceToTarget());
  }

  /**
   * Checks if a target is visible to the LimeLight.
   * 
   * @return True if a target is visible, otherwise false.
   */
  public boolean isTargetVisible() {
    return tv.getDouble(0.0) == 1.0;
  }

  /**
   * Returns the horizontal angle offset to the target.
   * 
   * @return Horizontal angle offset in degrees.
   */
  public double getHorizontalOffset() {
    return tx.getDouble(0.0);
  }

  /**
   * Returns the vertical angle offset to the target.
   * 
   * @return Vertical angle offset in degrees.
   */
  public double getVerticalOffset() {
    return ty.getDouble(0.0);
  }

  /**
   * Calculates the distance to the target based on the vertical angle offset.
   * 
   * @return Distance to the target in meters.
   */
  public double calculateDistanceToTarget() {
    double angleToTarget = Math.toRadians(LIMELIGHT_MOUNT_ANGLE + getVerticalOffset());
    return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(angleToTarget);
  }

  /**
   * Calculates the pose of the target relative to the robot.
   * 
   * @return Pose2d representing the position and orientation of the target relative to the robot.
   */
  public Pose2d calculateTargetPose() {
    double distance = calculateDistanceToTarget();
    double angle = Math.toRadians(getHorizontalOffset());
    double x = distance * Math.cos(angle);
    double y = distance * Math.sin(angle);
    return new Pose2d(x, y, new Rotation2d(0));
  }
}
