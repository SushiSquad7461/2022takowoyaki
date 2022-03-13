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
  // set tunning mode to true to enable tuning values over NT
  public static final boolean TUNNING_MODE = true;

  // the unit of measurement for Talon FX encoder velocity is known as the "Tran"
  // encoder ticks per 100ms
  public static double convertRPMtoTrans(double RPM) {
    return RPM * 2048.0 / 600.0;
  }

  // used for unit conversions for ff constants on talon fx
  public static double voltageToPercent(double voltage) {
    return voltage / 12.0;
  }

  public static final class kClimb {
    public static final int CLIMB_TO_TOP_BUTTON = XboxController.Button.kY.value;
    public static final int CLIMB_TO_BOTTOM_BUTTON = XboxController.Button.kA.value;
    public static final int CLIMB_LEFT_OPEN_LOOP_RAISE_BUTTON = XboxController.Button.kB.value;
    public static final int CLIMB_RIGHT_OPEN_LOOP_LOWER_BUTTON = XboxController.Button.kX.value;
    public static final int CLIMB_LEFT_OPEN_LOOP_LOWER_BUTTON = XboxController.Button.kX.value;
    public static final int CLIMB_ENCODER_RESET_BUTTON = XboxController.Button.kStart.value;
    public static final int SEPARATE_CLIMB = XboxController.Button.kLeftBumper.value;
    public static final int REJOIN_CLIMB = XboxController.Button.kRightBumper.value;

    // public static final int LEFT_MOTOR_CAN_ID = 15; // green climb
    public static final int LEFT_MOTOR_CAN_ID = 2; // blue
    public static final int RIGHT_MOTOR_CAN_ID = 17;

    public static final double OPEN_LOOP_UP_POWER = 1;
    public static final double OPEN_LOOP_DOWN_POWER = -1;

    public static final double CLOSED_LOOP_UP_POWER = -0.5;
    public static final double CLOSED_LOOP_DOWN_POWER = 0.5;

    // TODO: identify correct setpoints
    public static final int TOP_ENCODER_VAL = -165000;
    public static final int BOTTOM_ENCODER_VAL = -3000;

    public static final double OPEN_LOOP_RAMP_RATE = 0.5;
  }

  public static final class kOI {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // drive constants
    public static final int INVERT_DRIVE = XboxController.Button.kY.value;

    // hopper buttons
    public static final int SHOOT = XboxController.Button.kB.value;
    public static final int REVERSE_SHOOT = XboxController.Button.kBack.value;

    // intake buttons
    public static final int TOGGLE_INTAKE = XboxController.Button.kA.value;
    public static final int RUN_INTAKE = XboxController.Button.kA.value;
    public static final int REVERSE_INTAKE = XboxController.Button.kStart.value;

    // shooter buttons
    public static final int REV_SHOOTER = XboxController.Button.kB.value;

    public static final String TRAJECTORY_NAME = "path";
  }

  public static final class kHopper {
    public static int MOTOR_ID;
    public static boolean INVERTED;
    public static final int CURRENT_LIMIT = 15;
    public static final double SPEED = 1;

    public static final double OPEN_LOOP_RAMP_RATE = 0;
    public static final double JERKINESS = 100;
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

    // char values for bear metal carpet
    public static final double ksVolts = 0.71472; // 0.66412
    public static final double kvVoltSecondsPerMeter = 2.3953;
    public static final double kaVoltSecondsSquaredPerMeter = 0.21126; // 0.23884
    public static final double kPDriveVel = 0.000016636;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    // char values for garage carpet
    // public static final double ksVolts = 0.66858; // 0.66412
    // public static final double kvVoltSecondsPerMeter = 2.3302; // 1.6846
    // public static final double kaVoltSecondsSquaredPerMeter = 0.36796; // 0.23884
    // public static final double kPDriveVel = 0.0000015469;
    // public static final double kIDrive = 0;
    // public static final double kDDrive = 0;

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
    public static final double OPEN_LOOP_RAMP_RATE = 0.5; // 0.3
    public static final double CLOSED_LOOP_RAMP_RATE = 0.1;
    public static final double QUICKTURN_DAMPENER = 3; // bigger number = slower turns
  }

  public static final class kShooter {

    public static final class kOpenLoop {
      public static final double SPEED = 1;
      public static final double BACK_SPEED = 1; // only used on double shooters
    }

    public static final class kSingleClosedLoop {
      public static final double SETPOINT = convertRPMtoTrans(3400.0);

      public static double kP = 0.2;
      public static double kI = 0.0000;
      public static double kD = .0;
      public static double kF = 0.05;
      public static double kS = voltageToPercent(0.61716);
      public static double kV = voltageToPercent(0.10724);
      public static double kA = voltageToPercent(0.0082862);

    }

    public static final class kDoubleClosedLoop {
      public static final class kFront {
        // OUTREACH CONSTANT
        public static final double SETPOINT_RPM = 1400.0; // 1100 // 1065 COMP CONSTANT
        public static final double RANGED_SETPOINT = 1480;
        public static final double ERROR_TOLERANCE = 30;
        // public static final double SETPOINT_OFFSET_RPM = -30 + 100;
        public static final double SETPOINT_OFFSET_RPM = 0;
        public static double kP;
        public static double kI;
        public static double kD;
        public static double kF;
        public static double kS = voltageToPercent(0.61716);
        public static double kV = voltageToPercent(0.10724);
        public static double kA = voltageToPercent(0.0082862);
      }

      public static final class kBack {
        // OUTREACH CONSTANT
        public static final double SETPOINT_RPM = 3115.0; // 3250 // 3215 CCOMP CONSTANT
        public static final double RANGED_SETPOINT = 3445;
        public static final double ERROR_TOLERANCE = 30;
        // public static final double SETPOINT_OFFSET_RPM = 100.0 + 170.0;
        public static final double SETPOINT_OFFSET_RPM = 0;
        public static double kP;
        public static double kI;
        public static double kD;
        public static double kF;
        public static double kS;
        public static double kV;
        public static double kA;

      }
    }

    public static int LEFT_MOTOR_ID;
    public static int RIGHT_MOTOR_ID;
    public static int KICKER_MOTOR_ID;
    public static int BACK_MOTOR_ID;
    public static final int CURRENT_LIMIT = 35;
    public static final int DEFAULT_PROFILE_SLOT = 0;
    public static final int DEFAULT_CONFIG_TIMEOUT = 100;
    public static final double ERROR_TOLERANCE_PERCENT = 0.97;
    public static boolean KICKER_INVERSION;

    public static final double SPEED_KICKER = 1;
    public static final double KICKER_PERIOD = 50;
    public static final double KICKER_OFFSET = 0.3;
  }

  enum RobotType {
    PRACTICE,
    COMP
  }

  public static void setup() {
    RobotType robot = getRobotType();
    switch (robot) {
      case PRACTICE:
        // kHopper.MOTOR_ID = 10;
        // kIntake.MOTOR_ID = 8;
        // kIntake.SOLENOID_FRONT = 1;
        // kIntake.SOLENOID_BACK = 0;
        // kDrive.FRONT_RIGHT_ID = 3;
        // kDrive.FRONT_LEFT_ID = 1;
        // kDrive.BACK_RIGHT_ID = 4;
        // kDrive.BACK_LEFT_ID = 2;
        // kShooter.LEFT_MOTOR_ID = 12;
        // kShooter.RIGHT_MOTOR_ID = 15;
        // kShooter.KICKER_MOTOR_ID = 5;
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
        kShooter.kDoubleClosedLoop.kFront.kP = 0.15;
        kShooter.kDoubleClosedLoop.kFront.kI = 0.0000;
        kShooter.kDoubleClosedLoop.kFront.kD = 0.0;
        kShooter.kDoubleClosedLoop.kFront.kF = 0.045;
        kHopper.INVERTED = false;
        kShooter.KICKER_INVERSION = true;
        break;
      default:
        kHopper.MOTOR_ID = 0;
        kIntake.MOTOR_ID = 9;
        kIntake.LEFT_SOLENOID_FORWARD = 15;
        kIntake.LEFT_SOLENOID_REVERSE = 1;
        kIntake.RIGHT_SOLENOID_FORWARD = 0;
        kIntake.RIGHT_SOLENOID_REVERSE = 14;
        kDrive.FRONT_RIGHT_ID = 3;
        kDrive.FRONT_LEFT_ID = 16;
        kDrive.BACK_RIGHT_ID = 4;
        kDrive.BACK_LEFT_ID = 15;
        kShooter.LEFT_MOTOR_ID = 5;
        kShooter.RIGHT_MOTOR_ID = 14;
        kShooter.KICKER_MOTOR_ID = 10;
        kShooter.BACK_MOTOR_ID = 19;
        kShooter.kDoubleClosedLoop.kFront.kP = 0.075;
        kShooter.kDoubleClosedLoop.kFront.kI = 0.0000;
        kShooter.kDoubleClosedLoop.kFront.kD = 0.0;
        kShooter.kDoubleClosedLoop.kFront.kF = 0.05;
        kShooter.kDoubleClosedLoop.kBack.kP = 0.125;
        kShooter.kDoubleClosedLoop.kBack.kI = 0;
        kShooter.kDoubleClosedLoop.kBack.kD = 0;
        kShooter.kDoubleClosedLoop.kBack.kF = 0.045;
        kShooter.kDoubleClosedLoop.kBack.kS = voltageToPercent(0.0070982);
        kShooter.kDoubleClosedLoop.kBack.kV = voltageToPercent(0.0011253);
        kShooter.kDoubleClosedLoop.kBack.kA = voltageToPercent(0.000047908);
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
