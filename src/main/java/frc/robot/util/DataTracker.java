package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class provides logging functionalities for the FRC robot. It allows
 * logging of various types of data and manages file writing and smart dashboard
 * updates.
 */
public class DataTracker {

  /**
   * Logs a boolean value with specified group and key.
   *
   * @param groupId The group ID for categorizing the log entry.
   * @param key     The key identifying the log entry.
   * @param value   The boolean value to log.
   * @return True if the value was successfully sent to SmartDashboard, otherwise false.
   */
  public static boolean putBoolean(String groupId, String key, boolean value) {
    return SmartDashboard.putBoolean(groupId + key, value);
  }

  /**
   * Retrieves a boolean value from SmartDashboard using the specified group and key.
   *
   * @param groupId The group ID for categorizing the log entry.
   * @param key     The key identifying the log entry.
   * @param defaultValue The default value to return if the key does not exist.
   * @return The boolean value from SmartDashboard, or the default value if not found.
   */
  public static boolean getBoolean(String groupId, String key, boolean defaultValue) {
    return SmartDashboard.getBoolean(groupId + key, defaultValue);
  }

  /**
   * Logs a numeric value with specified group and key.
   *
   * @param groupId              The group ID for categorizing the log entry.
   * @param key                  The key identifying the log entry.
   * @param value                The numeric value to log.
   */
  public static void putNumber(String groupId, String key, double value) {
    SmartDashboard.putNumber(groupId + key, value);
  }

  /**
   * Retrieves a numeric value from SmartDashboard using the specified group and key.
   *
   * @param groupId The group ID for categorizing the log entry.
   * @param key     The key identifying the log entry.
   * @param defaultValue The default value to return if the key does not exist.
   * @return The numeric value from SmartDashboard, or the default value if not found.
   */
  public static double getNumber(String groupId, String key, double defaultValue) {
    return SmartDashboard.getNumber(groupId + key, defaultValue);
  }

  /**
   * Logs a string value with specified group and key.
   *
   * @param groupId              The group ID for categorizing the log entry.
   * @param key                  The key identifying the log entry.
   * @param value                The string value to log.
   */
  public static void putString(String groupId, String key, String value) {
    SmartDashboard.putString(groupId + key, value);
  }

  /**
   * Retrieves a string value from SmartDashboard using the specified group and key.
   *
   * @param groupId The group ID for categorizing the log entry.
   * @param key     The key identifying the log entry.
   * @param defaultValue The default value to return if the key does not exist.
   * @return The string value from SmartDashboard, or the default value if not found.
   */
  public static String getString(String groupId, String key, String defaultValue) {
    return SmartDashboard.getString(groupId + key, defaultValue);
  }
}
