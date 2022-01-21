// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class kShooter {
    public static final int LEFT_MOTOR_ID = 15;
    public static final int RIGHT_MOTOR_ID = 14;
    public static final double SPEED = 0.9;
    public static final int CURRENT_LIMIT = 35;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double GOAL = 3000;
    public static final int DEFAULT_PROFILE_SLOT = 0;
    public static final int DEFAULT_CONFIG_TIMEOUT = 100;

    public static final int RUN_SHOOTER = XboxController.Button.kX.value;
  }  
  public static final class kHopper {
    public static final int MOTOR_ID = 10;
    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 30;
    public static final double SPEED = 0.5;

    public static final int RUN_HOPPER = XboxController.Button.kX.value;
    public static final int REVERSE_HOPPER = XboxController.Button.kB.value;
  }
  
  public static final class kOI {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
  }
}
