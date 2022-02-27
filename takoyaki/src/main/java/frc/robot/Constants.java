  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public static final String TRAJECTORY_NAME = "path";
  }

  public static final class kHopper {
    public static int MOTOR_ID;
    public static boolean INVERTED;
    public static final int CURRENT_LIMIT = 30;
    public static final double SPEED = 1;

    public static final double OPEN_LOOP_RAMP_RATE = 0;
    public static final long JERKINESS = 50;
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

    // to divide quick turn power by
    public static final double QUICK_TURN_DAMPENER = 3.0;

    // current limits
    public static final double SUPPLY_CURRENT_LIMIT = 30;
    public static final double STATOR_CURRENT_LIMIT = 30;

    // char values for garage carpet
    public static final double ksVolts = 0.66858; // 0.66412
    public static final double kvVoltSecondsPerMeter = 2.3302; // 1.6846
    public static final double kaVoltSecondsSquaredPerMeter = 0.36796; // 0.23884
    public static final double kPDriveVel = 0.0000015469;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    // char values for garage
    // public static final double ksVolts = 0.54849;
    // public static final double kvVoltSecondsPerMeter = 1.6912;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.21572;
    // public static final double kPDriveVel = 0.00005;
    // public static final double kIDrive = 0;
    // public static final double kDDrive = 0;

    // public static final double kPDriveVel = 0;
    public static final double MAX_VOLTAGE = 5;

    // odometry constants - drivetrain measurements
    public static final double TRACK_WIDTH_METERS = 0.69; // width between sides of dt
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    // path-following constants
    public static final double MAX_SPEED_METERS_PER_SECOND = 5; // set to somewhat below free speed
    // could increase this to go faster
    // theoretically
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 5; // doesn't really matter

    // ticks to meters conversion factor for falcon 500
    // (total ticks) * (motor rotations/tick) * (wheel rotations/motor rotations) *
    // (meters/wheel rotations)
    public static final double TICKS_TO_METERS = (1.0 / 2048.0) * (1.0 / 10.71) * (0.4788);

    // ramsete parameters
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static final class kShooter {
    public static final class kOpenLoop {
      public static final double SETPOINT = 1;
    }

    public static final class kClosedLoop {
      public static final double SETPOINT = 3225.0 * 2048.0 / 600.0;
      public static final double ERROR_TOLERANCE = 15.0 * 2048.0 / 600.0;
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
  enum RobotType{
    PRACTICE,
    COMP
  }
  public static void setup() {
    RobotType robot = getRobotType();
    switch(robot) {
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
          kHopper.INVERTED = false;
          kShooter.KICKER_INVERSION = true;
          break;
        default:
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
    //Map<String, String> env = System.getenv();
    //SmartDashboard.putString("Home", env.get("HOME"));
    File f = new File("/home/lvuser/id.txt");
    int id = 0;
    String errorMsg = "success";
    try {
      Scanner reader = new Scanner(f);
      id = reader.nextInt();
      reader.close();
    } catch (FileNotFoundException e){
      errorMsg="file not found exception";
      SmartDashboard.putString("robot type status", errorMsg);
    }
    if(id == 1) {
      SmartDashboard.putString("robot", "practice");
      return RobotType.PRACTICE;
    } else {
      SmartDashboard.putString("robot", "comp");
      return RobotType.COMP;
    }
    
  }
}
