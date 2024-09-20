package frc.robot.util.hardware;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.constants.Device;

import java.util.function.Supplier;

/**
 * A wrapper class for the CANSparkFlex that adds support for CANDevice and includes retry logic.
 */
public class SparkFlex extends CANSparkFlex {

  private static final int MAX_RETRIES = 5;

  private static final double INITIAL_RETRY_DELAY = 0.3; // Initial delay in seconds
  private static final double MAX_RETRY_DELAY = 2.0; // Maximum delay in seconds
  private static final double BACKOFF_MULTIPLIER = 2.0; // Multiplier for each retry

  private CANDevice canDevice;

  /**
   * Constructs a new SparkFlex object with the specified CANDevice.
   *
   * @param canDevice CANDevice object containing device information.
   */
  public SparkFlex(CANDevice canDevice) {
    super(canDevice.getCanId(), MotorType.kBrushless);
    this.canDevice = canDevice;

    // Restore factory defaults
    restoreFactoryDefaults();

    // Enable voltage compensation to 12V
    enableVoltageCompensation(12.0);
  }

  // Accessor methods for CANDevice properties

  /**
   * Returns the subsystem of the CANDevice.
   *
   * @return Subsystem name.
   */
  public String getSubsystem() {
    return canDevice.getSubsystem();
  }

  /**
   * Returns the device name of the CANDevice.
   *
   * @return Device name.
   */
  public String getDeviceName() {
    return canDevice.getDeviceName();
  }

  /**
   * Returns the CAN ID of the CANDevice.
   *
   * @return CAN ID.
   */
  public int getCanId() {
    return canDevice.getCanId();
  }

  /**
   * Returns the Device enum of the CANDevice.
   *
   * @return Device enum.
   */
  public Device getDevice() {
    return canDevice.getDevice();
  }

  // #region CANSparkLowLevel overrides

  @Override
  public REVLibError restoreFactoryDefaults() {
    return applyCommandWithRetry(super::restoreFactoryDefaults, "restoreFactoryDefaults");
  }

  // #endregion CANSparkLowLevel overrides

  // #region CANSparkBase overrides

  @Override
  public REVLibError setSmartCurrentLimit(int limit) {
    return applyCommandWithRetry(() -> super.setSmartCurrentLimit(limit), "setSmartCurrentLimit");
  }

  @Override
  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
    return applyCommandWithRetry(() -> super.setSmartCurrentLimit(stallLimit, freeLimit), "setSmartCurrentLimit");
  }

  @Override
  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
    return applyCommandWithRetry(() -> super.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM), "setSmartCurrentLimit");
  }

  @Override
  public REVLibError setSecondaryCurrentLimit(double limit) {
    return applyCommandWithRetry(() -> super.setSecondaryCurrentLimit(limit), "setSecondaryCurrentLimit");
  }

  @Override
  public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
    return applyCommandWithRetry(() -> super.setSecondaryCurrentLimit(limit, chopCycles), "setSecondaryCurrentLimit");
  }

  @Override
  public REVLibError setIdleMode(IdleMode mode) {
    return applyCommandWithRetry(() -> super.setIdleMode(mode), "setIdleMode");
  }

  @Override
  public REVLibError enableVoltageCompensation(double nominalVoltage) {
    return applyCommandWithRetry(() -> super.enableVoltageCompensation(nominalVoltage), "enableVoltageCompensation");
  }

  @Override
  public REVLibError disableVoltageCompensation() {
    return applyCommandWithRetry(super::disableVoltageCompensation, "disableVoltageCompensation");
  }

  @Override
  public REVLibError setOpenLoopRampRate(double rate) {
    return applyCommandWithRetry(() -> super.setOpenLoopRampRate(rate), "setOpenLoopRampRate");
  }

  @Override
  public REVLibError setClosedLoopRampRate(double rate) {
    return applyCommandWithRetry(() -> super.setClosedLoopRampRate(rate), "setClosedLoopRampRate");
  }

  @Override
  public REVLibError follow(final CANSparkBase leader) {
    return applyCommandWithRetry(() -> super.follow(leader), "follow");
  }

  @Override
  public REVLibError follow(final CANSparkBase leader, boolean invert) {
    return applyCommandWithRetry(() -> super.follow(leader, invert), "follow");
  }

  @Override
  public REVLibError follow(ExternalFollower leader, int deviceID) {
    return applyCommandWithRetry(() -> super.follow(leader, deviceID), "follow");
  }

  @Override
  public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
    return applyCommandWithRetry(() -> super.follow(leader, deviceID, invert), "follow");
  }

  @Override
  public REVLibError clearFaults() {
    return applyCommandWithRetry(super::clearFaults, "clearFaults");
  }

  @Override
  public REVLibError burnFlash() {
    return applyCommandWithRetry(super::burnFlash, "burnFlash");
  }

  @Override
  public REVLibError setCANTimeout(int milliseconds) {
    return applyCommandWithRetry(() -> super.setCANTimeout(milliseconds), "setCANTimeout");
  }

  @Override
  public REVLibError enableSoftLimit(SoftLimitDirection direction, boolean enable) {
    return applyCommandWithRetry(() -> super.enableSoftLimit(direction, enable), "enableSoftLimit");
  }

  @Override
  public REVLibError setSoftLimit(SoftLimitDirection direction, float limit) {
    return applyCommandWithRetry(() -> super.setSoftLimit(direction, limit), "setSoftLimit");
  }

  @Override
  public REVLibError getLastError() {
    return applyCommandWithRetry(super::getLastError, "getLastError");
  }

  // #endregion CANSparkBase overrides

  /**
   * Executes a command with retry logic using exponential backoff. This is effective for transient errors.
   *
   * @param command    A {@link Supplier} of {@link REVLibError}, representing the command to execute.
   * @param methodName Name of the method for logging.
   * @return {@link REVLibError} status of the command. Returns success status on early success or the last error after max retries.
   *
   * The retry logic starts with an initial delay (INITIAL_RETRY_DELAY) and increases the delay after each failed attempt
   * by a factor of BACKOFF_MULTIPLIER, capped at MAX_RETRY_DELAY. A warning is logged on each failed attempt. If all attempts fail,
   * an error is logged.
   */
  private REVLibError applyCommandWithRetry(Supplier<REVLibError> command, String methodName) {
    REVLibError status = REVLibError.kUnknown;
    double currentDelay = INITIAL_RETRY_DELAY;

    for (int i = 0; i < MAX_RETRIES; i++) {
      status = command.get();
      if (status == REVLibError.kOk) {
        return status;
      }

      // Apply backoff strategy
      Timer.delay(currentDelay);
      currentDelay = Math.min(currentDelay * BACKOFF_MULTIPLIER, MAX_RETRY_DELAY);

      // Log retry attempt
      String retryLog = String.format("CANSparkFlex (%d): %s attempt %d failed, retrying in %.2f seconds",
                                      this.getDeviceId(), methodName, i + 1, currentDelay);
      DriverStation.reportError(retryLog, false);
    }

    String error = String.format("CANSparkFlex (%d): %s failed after %d attempts",
                                 this.getDeviceId(), methodName, MAX_RETRIES);
    DriverStation.reportError(error, false);
    return status;
  }
}
