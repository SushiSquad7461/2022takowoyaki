// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.kDrive.kClosedLoop.kLeft;
import frc.robot.Constants.kDrive.kClosedLoop.kRight;

public class Constants {

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
  }

  public static final class kHopper {
    public static int MOTOR_ID;
    public static boolean INVERTED;
    public static final int CURRENT_LIMIT = 30;
    public static final double SPEED = 0.7;

    public static final double OPEN_LOOP_RAMP_RATE = 0;
    public static final long JERKINESS = 00;
  }

  public static final class kIntake {
    public static final class kFalcon {
      public static final TalonFXInvertType INVERT_TYPE = TalonFXInvertType.CounterClockwise;
      public static final TalonFXControlMode CONTROL_MODE = TalonFXControlMode.PercentOutput;
    }

    public static final class kSpark {
      public static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
      public static final int CURRENT_LIMIT = 35;
      public static final int OPEN_LOOP_RAMP_RATE = 0;
      public static final boolean INVERTED = true;
    }

    public static int MOTOR_ID;
    public static int LEFT_SOLENOID_FORWARD;
    public static int LEFT_SOLENOID_REVERSE;
    public static int RIGHT_SOLENOID_FORWARD;
    public static int RIGHT_SOLENOID_REVERSE;

    public static final double INTAKE_SPEED = 0.9;
  }

  public static final class kDrive {
    public static int FRONT_RIGHT_ID;
    public static int FRONT_LEFT_ID;
    public static int BACK_RIGHT_ID;
    public static int BACK_LEFT_ID;

    public static final class kClosedLoop {
      public static final class kLeft {
        public static double kP;
        public static double kI;
        public static double kD;

        // Feedforward constants
        public static double kF;
        public static double kS;
        public static double kV;
        public static double kA;
      }

      public static final class kRight {
        public static double kP;
        public static double kI;
        public static double kD;

        // Feedforward constants
        public static double kF;
        public static double kS;
        public static double kV;
        public static double kA;
      }
    }
  }

  public static final class kShooter {
    public static final class kOpenLoop {
      public static final double SETPOINT = 1;
    }

    public static final class kClosedLoop {
      public static final double SETPOINT = 3400.0 * 2048.0 / 600.0;
    }

    public static int LEFT_MOTOR_ID;
    public static int RIGHT_MOTOR_ID;
    public static int KICKER_MOTOR_ID;
    public static final double SPEED = 0.9;
    public static final int CURRENT_LIMIT = 35;
    public static double kP = 0.2;
    public static double kI = 0.0000;
    public static double kD = .0;
    public static double kF = 0.05;
    public static double kS = 0.61716 / 12.0;
    public static double kV = 0.10724 / 12.0;
    public static double kA = 0.0082862 / 12.0;
    public static final int DEFAULT_PROFILE_SLOT = 0;
    public static final int DEFAULT_CONFIG_TIMEOUT = 100;

    public static boolean KICKER_INVERSION;

    public static final double SPEED_KICKER = 1;
  }

  enum RobotType {
    PRACTICE,
    COMP
  }

  public static void setup() {
    RobotType robot = getRobotType();
    switch (robot) {
      case PRACTICE:
        kHopper.MOTOR_ID = 10;

        kIntake.MOTOR_ID = 8;
        kIntake.LEFT_SOLENOID_FORWARD = -1;
        kIntake.LEFT_SOLENOID_REVERSE = -1;
        kIntake.RIGHT_SOLENOID_FORWARD = -1;
        kIntake.RIGHT_SOLENOID_REVERSE = -1;

        kDrive.FRONT_RIGHT_ID = 3;
        kDrive.FRONT_LEFT_ID = 1;
        kDrive.BACK_RIGHT_ID = 4;
        kDrive.BACK_LEFT_ID = 2;

        kShooter.LEFT_MOTOR_ID = 12;
        kShooter.RIGHT_MOTOR_ID = 15;
        kShooter.KICKER_MOTOR_ID = 5;
        kShooter.kP = 0.15;
        kShooter.kI = 0.0000;
        kShooter.kD = 0.0;
        kShooter.kF = 0.045;
        kShooter.KICKER_INVERSION = true;

        kHopper.INVERTED = false;
        break;
      default: // comp bot
        kHopper.MOTOR_ID = 10;

        kIntake.MOTOR_ID = 9;
        kIntake.LEFT_SOLENOID_FORWARD = 14;
        kIntake.LEFT_SOLENOID_REVERSE = 15;
        kIntake.RIGHT_SOLENOID_FORWARD = 2;
        kIntake.RIGHT_SOLENOID_REVERSE = 1;

        kDrive.FRONT_RIGHT_ID = 15;
        kDrive.FRONT_LEFT_ID = 4;
        kDrive.BACK_RIGHT_ID = 16;
        kDrive.BACK_LEFT_ID = 3;
        kRight.kP = 0.2;
        kRight.kI = 0.0;
        kRight.kD = 0.0;
        kRight.kF = 0.0;
        kRight.kS = 0.0;
        kRight.kA = 0.0;
        kRight.kV = 0.0;
        kLeft.kP = 0.2;
        kLeft.kI = 0.0;
        kLeft.kD = 0.0;
        kLeft.kF = 0.0;
        kLeft.kS = 0.0;
        kLeft.kA = 0.0;
        kLeft.kV = 0.0;

        kShooter.LEFT_MOTOR_ID = 5;
        kShooter.RIGHT_MOTOR_ID = 14;
        kShooter.KICKER_MOTOR_ID = 0;
        kShooter.kP = 0.20;
        kShooter.kI = 0.0000;
        kShooter.kD = 0.0;
        kShooter.kF = 0.05;

        kHopper.INVERTED = true;
        kShooter.KICKER_INVERSION = false;
        break;
    }
  }

  public static RobotType getRobotType() {
    // Map<String, String> env = System.getenv();
    // SmartDashboard.putString("Home", env.get("HOME"));
    File f = new File("/home/lvuser/id.txt");
    int id = 0;
    String errorMsg = "success";
    try {
      Scanner reader = new Scanner(f);
      id = reader.nextInt();
      reader.close();
    } catch (FileNotFoundException e) {
      errorMsg = "file not found exception";
      SmartDashboard.putString("robot type status", errorMsg);
    }
    if (id == 1) {
      SmartDashboard.putString("robot", "practice");
      return RobotType.PRACTICE;
    } else {
      SmartDashboard.putString("robot", "comp");
      return RobotType.COMP;
    }

  }
}
