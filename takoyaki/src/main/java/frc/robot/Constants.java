// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
  
  public static final class kOI {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    
    // hopper buttons
    public static final int RUN_HOPPER = XboxController.Button.kRightBumper.value;
    public static final int REVERSE_HOPPER = XboxController.Button.kBack.value;

    // intake buttons    
    public static final int TOGGLE_INTAKE = XboxController.Button.kLeftBumper.value;

    public static final int RUN_INTAKE = XboxController.Button.kA.value;
    public static final int REVERSE_INTAKE = XboxController.Button.kStart.value;

    // shooter buttons
    public static final int RUN_SHOOTER = XboxController.Button.kB.value;

    public static final String TRAJECTORY_NAME = "path";
  }
  
  public static final class kHopper {
    public static final int MOTOR_ID = 10;
    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 30;
    public static final double SPEED = 0.9;

    public static final double OPEN_LOOP_RAMP_RATE = 0;
  }
  
  public static final class kIntake {
    public static final int MOTOR_ID = 8;
    public static final int SOLENOID_FRONT = 1;
    public static final int SOLENOID_BACK = 0;
    public static final float INTAKE_SPEED = 0.9f;

    public static final int CURRENT_LIMIT = 35;
    public static final int OPEN_LOOP_RAMP_RATE = 0;
    public static final boolean INVERTED = true;
  }

  public static final class kDrive {
    public static final int FRONT_LEFT_ID = 1;
    public static final int BACK_LEFT_ID = 2;
    public static final int FRONT_RIGHT_ID = 3;
    public static final int BACK_RIGHT_ID = 4;

    //  to divide quick turn power by
    public static final double QUICK_TURN_DAMPENER = 3.0; 

    // char values for garage
    public static final double ksVolts = 0.54849;
    public static final double kvVoltSecondsPerMeter = 1.6912;
    public static final double kaVoltSecondsSquaredPerMeter = 0.21572;
    public static final double kPDriveVel = 0.00005;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    // public static final double kPDriveVel = 0;
    public static final double MAX_VOLTAGE = 5;

    // odometry constants - drivetrain measurements
    public static final double TRACK_WIDTH_METERS = 0.69; // width between sides of dt
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = 
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // path-following constants
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; // set to somewhat below free speed
                                                              // could increase this to go faster 
                                                              // theoretically
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3; // doesn't really matter

    // ticks to meters conversion factor for falcon 500
    // (total ticks) * (motor rotations/tick) * (wheel rotations/motor rotations) * (meters/wheel rotations)
    public static final double TICKS_TO_METERS = (1.0/2048.0) * (1.0/7.31) * (0.4788);

    // ramsete parameters
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static final class kShooter {
    public static final int LEFT_MOTOR_ID = 12;
    public static final int RIGHT_MOTOR_ID = 15;
    public static final int KICKER_MOTOR_ID = 5;
    public static final double SPEED = 0.9;
    public static final int CURRENT_LIMIT = 35;
    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.045;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double GOAL = 3000;
    public static final int DEFAULT_PROFILE_SLOT = 0;
    public static final int DEFAULT_CONFIG_TIMEOUT = 100;

    public static final boolean KICKER_INVERSION = true;

    public static final double SPEED_KICKER = 1;
  }  
}
