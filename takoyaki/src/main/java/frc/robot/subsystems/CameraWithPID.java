package frc.robot.subsystems;

import frc.robot.subsystems.Camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;

public class CameraWithPID {
  private final Camera camera;
  private final PIDController controller;

  public CameraWithPID(String name, PIDController controller) {
    this.camera = new Camera(name);
    this.controller = controller;
  }
  
  public void alignToTarget() {
    try {
      if (camera.hasTargets()) {
        controller.setSetpoint(camera.getYaw());
      }
    } catch (Exception e) {
      SmartDashboard.putString("Errors", e.getMessage());
    }
  }
}
