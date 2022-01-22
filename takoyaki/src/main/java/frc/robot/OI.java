package frc.robot;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
  // returns difference between Right trigger value and Left trigger value
  public static double getTriggers(XboxController controller) {
    double val = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    return Math.pow(val, 3);
  }

  public static double getLeftStick(XboxController controller) {
    return Math.pow(controller.getLeftX(), 3);
  }

  public static double getRightStick(XboxController controller) {
    return Math.pow(controller.getRightY(), 3);
  }
}