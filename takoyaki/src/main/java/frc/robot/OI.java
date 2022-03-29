package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
  // returns difference between Right trigger value and Left trigger value
  public static double getTriggers(XboxController controller) {
    double val = controller.getRightTriggerAxis() -
        controller.getLeftTriggerAxis();
    return Math.pow(val, 1);
  }

  // returns trigger output but the speed is capped (no cap!!)
  private static double lastTriggerSpeed = 0;

  /*
   * public static double getTriggersProportionalScaling(XboxController
   * controller) {
   * double val = controller.getRightTriggerAxis() -
   * controller.getLeftTriggerAxis();
   * double error = val - lastTriggerSpeed;
   * double change = error * Constants.kOI.TRIGGER_SPEED_PROPORTIONAL;
   * if (change > Constants.kOI.MAX_ACCELL) {
   * change = Constants.kOI.MAX_ACCELL;
   * }
   * double toReturn = lastTriggerSpeed + change;
   * lastTriggerSpeed = toReturn;
   * return Math.pow(toReturn, 1);
   * }
   */

  public static double getLeftStick(XboxController controller) {
    return Math.pow(controller.getLeftX(), 3);
  }

  public static double getRightStick(XboxController controller) {
    return Math.pow(controller.getRightY(), 3);
  }
}