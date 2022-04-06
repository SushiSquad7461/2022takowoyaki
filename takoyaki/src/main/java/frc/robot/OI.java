package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
  // returns difference between right trigger value and left trigger value
  public static double getTriggers(XboxController controller) {
    double val = controller.getRightTriggerAxis() -
        controller.getLeftTriggerAxis();
    return Math.pow(val, 1);
  }

  public static double getLeftStickX(XboxController controller) {
    return Math.pow(controller.getLeftX(), 3);
  }

  public static double getLeftStickY(XboxController controller) {
    return controller.getLeftY();
  }

  public static double getRightStickY(XboxController controller) {
    return controller.getRightY();
  }

  public static boolean joystickActive(XboxController controller) {
    return (Math.abs(controller.getLeftY()) > 0.1 || Math.abs(controller.getRightY()) > 0.1);
  }
}