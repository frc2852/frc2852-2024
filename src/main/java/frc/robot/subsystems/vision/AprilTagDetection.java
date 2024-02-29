package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.vision.CameraConfiguration;

/**
 * Subsystem for detecting AprilTags using PhotonVision.
 * This class manages a PhotonCamera and uses it to detect and estimate positions
 * of AprilTags in the camera's field of view.
 */
public class AprilTagDetection extends SubsystemBase {

  private final PhotonCamera camera;
  private final CameraConfiguration cameraConfig;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;
  private final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  /**
   * Constructs a new instance of AprilTagDetectionSubsystem.
   * This constructor initializes the PhotonVision camera and pose estimation system using the provided camera configuration.
   * It ensures the camera configuration is not null and sets up the necessary components for AprilTag detection.
   *
   * @param cameraConfig The configuration settings for the camera.
   * @throws IllegalArgumentException if the cameraConfig parameter is null.
   */
  public AprilTagDetection(CameraConfiguration cameraConfig) {
    if (cameraConfig == null) {
      throw new IllegalArgumentException("Camera configuration cannot be null.");
    }

    this.cameraConfig = cameraConfig;
    this.camera = initializePhotonVisionCamera();
    initializePoseEstimation();
  }

  /**
   * Initializes the PhotonVision camera.
   * <p>
   * This method creates a new PhotonCamera instance using the camera name from the camera configuration.
   * It checks if the camera is connected and reports an error to the DriverStation if the connection fails.
   * If the camera is successfully connected, the driver mode is set to false, and the LED mode is turned off.
   * 
   * @return The initialized PhotonCamera if successful, or null if the camera is not connected.
   */
  private PhotonCamera initializePhotonVisionCamera() {
    PhotonCamera newCamera = new PhotonCamera(cameraConfig.getName());
    newCamera.setDriverMode(false);
    newCamera.setLED(VisionLEDMode.kOff);
    return newCamera;
  }

  /**
   * Initializes the pose estimation system using AprilTag field layouts.
   * It sets up the field layout and the pose estimator using the camera configuration.
   * If an IOException occurs during setup, an error is reported via the DriverStation.
   */
  private void initializePoseEstimation() {
    try {
      // Load the AprilTag field layout from a resource file.
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

      // Set the origin position for the field layout.
      aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

      // Initialize the pose estimator with the loaded field layout, strategy, camera, and camera position.
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, camera,
          cameraConfig.getPosition());
    } catch (IOException e) {
      // Report the error to the DriverStation without halting the program.
      DriverStation.reportError("Failed to load AprilTag field layout.", false);
    }
  }

  /**
   * Periodically checks and updates the robot's status based on the camera's connectivity.
   * If the camera is connected, it retrieves the latest result from the photon pipeline
   * and updates the visible tags. Otherwise, it reports a warning to the DriverStation.
   */
  @Override
  public void periodic() {
    if (!DriverStation.isEnabled())
      return;

    // Check if the camera is connected before attempting to process the data.
    if (camera == null || !camera.isConnected()) {
      DriverStation.reportWarning("Camera is not connected. Cannot update AprilTags.", false);
    }
  }

  /**
   * Estimates the global pose of the robot using the photon pose estimator.
   * It sets the reference pose for the estimator and then updates to get the estimated pose.
   *
   * @param prevEstimatedRobotPose The previously estimated pose of the robot.
   * @return An Optional containing the EstimatedRobotPose, if available.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

}
