package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfiguration {
  private final String name;
  private final Transform3d position;
  private final double pitch;

  public CameraConfiguration(String pipelineName, Transform3d position, double pitch) {
    this.name = pipelineName;
    this.position = position;
    this.pitch = pitch;
  }

  public String getName() {
    return name;
  }

  public Transform3d getPosition() {
    return position;
  }

  public double getHeight() {
    return position.getY();
  }

  public double getPitch() {
    return pitch;
  }
}
