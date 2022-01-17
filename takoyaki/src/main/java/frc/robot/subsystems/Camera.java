package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;


public class Camera {
  private PhotonCamera camera;
  private String name;
  public Camera(String name) {
    this.camera = new PhotonCamera(name);
    this.name = name;
  }

  public void periodic() {
    SmartDashboard.putBoolean("Camera \"" + name + "\" Sees Targets", camera.getLatestResult().hasTargets());
  }
  
  public boolean hasTargets() {
    return camera.getLatestResult().hasTargets();
  }

  public List<PhotonTrackedTarget> getAllTargets() throws NoTargetsException {
    if (!hasTargets()) {
      throw new NoTargetsException("Camera \"" + name + "\" has no targets");
    }
    return camera.getLatestResult().getTargets();
  }

  public PhotonTrackedTarget getBestTarget() throws NoTargetsException {
    if (!hasTargets()) {
      throw new NoTargetsException("Camera \"" + name + "\" has no targets");
    }
    return camera.getLatestResult().getBestTarget();
  }

  public double getDistance(double CAMERA_HEIGHT, double TARGET_HEIGHT, double PITCH_RADIANS) throws NoTargetsException {
    double range = PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT, 
      TARGET_HEIGHT,
      PITCH_RADIANS,
      Units.degreesToRadians(getBestTarget().getPitch()));
    return range;
  }

  public double getYaw() throws NoTargetsException {
    double yaw = getBestTarget().getYaw();
    return yaw;
  }

  class NoTargetsException extends Exception {
    public NoTargetsException (String message) {
      super(message);
    }
  }
}
