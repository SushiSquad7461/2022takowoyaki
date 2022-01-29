// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
  public static final class kIntake {
    public static final int FALCON_MOTOR_ID = -1;
    public static final int SOLENOID_FRONT = 1;
    public static final int SOLENOID_BACK = 0;
    public static final float INTAKE_SPEED = 0.9f;

    public static final int RETRACT_INTAKE = XboxController.Button.kY.value;
    public static final int ACTUATE_INTAKE  = XboxController.Button.kA.value;
    
    public static final int RUN_INTAKE = XboxController.Button.kX.value;
    public static final int REVERSE_INTAKE = XboxController.Button.kB.value;

public final class Constants {
  public static final class kDrive {
    public static final int FRONT_RIGHT_ID = 3;
    public static final int FRONT_LEFT_ID = 1;
    public static final int BACK_RIGHT_ID = 4;
    public static final int BACK_LEFT_ID = 2;
  }

  public static final class kOI {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
  }
}
