package frc.robot.util.swerve;

import frc.robot.constants.Constants.OperatorConstant;
import frc.robot.constants.SwerveConstants.SwerveDrive;

public class SwerveUtils {

  /**
   * Steps a value towards a target with a specified step size.
   *
   * @param _current  The current or starting value. Can be positive or negative.
   * @param _target   The target value the algorithm will step towards. Can be
   *                  positive or negative.
   * @param _stepsize The maximum step size that can be taken.
   * @return The new value for {@code _current} after performing the specified
   *         step towards the specified target.
   */
  public static double StepTowards(double _current, double _target, double _stepsize) {
      double delta = _target - _current;
      if (delta > _stepsize) {
          return _current + _stepsize;
      } else if (delta < -_stepsize) {
          return _current - _stepsize;
      } else {
          return _target;
      }
  }

  /**
   * Steps a value (angle) towards a target (angle) taking the shortest path with
   * a specified step size.
   *
   * @param _current  The current or starting angle (in radians). Can lie outside
   *                  the 0 to 2*PI range.
   * @param _target   The target angle (in radians) the algorithm will step
   *                  towards. Can lie outside the 0 to 2*PI range.
   * @param _stepsize The maximum step size that can be taken (in radians).
   * @return The new angle (in radians) for {@code _current} after performing the
   *         specified step towards the specified target.
   *         This value will always lie in the range 0 to 2*PI (exclusive).
   */
  public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
      _current = WrapAngle(_current);
      _target = WrapAngle(_target);

      double stepDirection = Math.signum(AngleWrappedDifference(_target, _current));
      double difference = AngleDifference(_current, _target);

      if (difference <= _stepsize) {
          return _target;
      } else {
          return WrapAngle(_current + stepDirection * _stepsize);
      }
  }

  /**
   * Finds the minimum difference between two angles
   *
   * @param _angleA An angle (in radians).
   * @param _angleB An angle (in radians).
   * @return The (unsigned) minimum difference between the two angles (in
   *         radians).
   */
  public static double AngleDifference(double angle1, double angle2) {
      return Math.abs(AngleWrappedDifference(angle1, angle2));
  }

  // Helper function to compute the signed minimal angular difference
  public static double AngleWrappedDifference(double angle1, double angle2) {
      double diff = angle1 - angle2;
      diff = (diff + Math.PI) % (2 * Math.PI) - Math.PI;
      return diff;
  }

  /**
   * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
   *
   * @param _angle The angle (in radians) to wrap. Can be positive or negative and
   *               can lie multiple wraps outside the output range.
   * @return An angle (in radians) from 0 and 2*PI (exclusive).
   */
  public static double WrapAngle(double _angle) {
      double wrapped = _angle % SwerveDrive.TWO_PI;
      if (wrapped < 0.0) {
          wrapped += SwerveDrive.TWO_PI;
      }
      return wrapped;
  }

  /**
   * Applies an exponential response curve to a joystick input value.
   *
   * This method modifies the input value according to an exponential curve,
   * making the response more sensitive at the extremes and less sensitive
   * around the center. The shape of the curve is controlled by the
   * {@link OperatorConstant#EXPONENTIAL_RESPONSE} constant.
   *
   * @param input The raw joystick input value, typically in the range [-1, 1].
   * @return The modified input value after applying the exponential response curve.
   */
  public static double applyExponentialResponse(double input) {
      return Math.signum(input) * Math.pow(Math.abs(input), OperatorConstant.EXPONENTIAL_RESPONSE);
  }
}