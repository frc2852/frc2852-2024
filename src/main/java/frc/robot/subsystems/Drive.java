package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CANBus;
import frc.robot.constants.SwerveConstants.SwerveDrive;
import frc.robot.util.swerve.SDSMK4iSwerveModule;
import frc.robot.util.swerve.SwerveUtils;

public class Drive extends SubsystemBase {

  // Create SDSMK4iSwerveModules
  private final SDSMK4iSwerveModule frontLeft = new SDSMK4iSwerveModule(
      CANBus.FRONT_LEFT_DRIVE,
      CANBus.FRONT_LEFT_TURN,
      CANBus.FRONT_LEFT_ENCODER,
      SwerveDrive.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SDSMK4iSwerveModule frontRight = new SDSMK4iSwerveModule(
      CANBus.FRONT_RIGHT_DRIVE,
      CANBus.FRONT_RIGHT_TURN,
      CANBus.FRONT_RIGHT_ENCODER,
      SwerveDrive.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SDSMK4iSwerveModule rearLeft = new SDSMK4iSwerveModule(
      CANBus.REAR_LEFT_DRIVE,
      CANBus.REAR_LEFT_TURN,
      CANBus.REAR_LEFT_ENCODER,
      SwerveDrive.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SDSMK4iSwerveModule rearRight = new SDSMK4iSwerveModule(
      CANBus.REAR_RIGHT_DRIVE,
      CANBus.REAR_RIGHT_TURN,
      CANBus.REAR_RIGHT_ENCODER,
      SwerveDrive.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // Sensors
  private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(SwerveDrive.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(SwerveDrive.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry for tracking robot pose
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      SwerveDrive.DRIVE_KINEMATICS,
      getRotation(),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      },
      new Pose2d(0, 0, new Rotation2d(0)) // 0 or 180
  );

  // Smartdashboard
  private final Field2d field = new Field2d();

  // Subsystems
  public Drive() {
    // Reset the gyro for field orientation
    zeroHeading();

    // Smartdashboard
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    poseEstimator.update(
        getRotation(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    Pose2d pose = poseEstimator.getEstimatedPosition();
    field.setRobotPose(pose);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to the specified pose.
   *
   * @param pose The pose to which to set the pose estimator.
   */
  public void resetPoseEstimator(Pose2d pose) {
    poseEstimator.resetPosition(getRotation(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        }, pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(SwerveDrive.DIRECTION_SLEW_RATE / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeedCommanded * SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = currentRotation * SwerveDrive.MAX_ANGULAR_SPEED;

    SwerveModuleState[] swerveModuleStates = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDrive.MAX_SPEED_METERS_PER_SECOND);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void lockDrive() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDrive.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navX.zeroYaw();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getHeadingDegrees() {
    try {
      return Math.IEEEremainder(navX.getAngle() * (SwerveDrive.GYRO_REVERSED ? -1.0 : 1.0), 360);
    } catch (Exception e) {
      DriverStation.reportError("Cannot Get NavX Heading", false);
      return 0;
    }
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate() * (SwerveDrive.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Returns the current robot-relative ChassisSpeeds.
   *
   * @return The current robot-relative ChassisSpeeds.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Retrieve the current states of each swerve module
    SwerveModuleState frontLeftState = frontLeft.getState();
    SwerveModuleState frontRightState = frontRight.getState();
    SwerveModuleState rearLeftState = rearLeft.getState();
    SwerveModuleState rearRightState = rearRight.getState();

    // Convert the swerve module states to chassis speeds
    return SwerveDrive.DRIVE_KINEMATICS.toChassisSpeeds(
        frontLeftState, frontRightState, rearLeftState, rearRightState);
  }

  /**
   * Drives the robot using robot-relative speeds.
   *
   * @param speeds The robot-relative ChassisSpeeds.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert the robot-relative speeds to swerve module states
    SwerveModuleState[] moduleStates = SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    // Desaturate the wheel speeds to ensure they are within the maximum speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveDrive.MAX_SPEED_METERS_PER_SECOND);

    // Set the desired state for each swerve module
    setModuleStates(moduleStates);
  }
}