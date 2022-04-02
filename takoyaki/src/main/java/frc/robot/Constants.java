// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {

  // set tuning mode to true to enable tuning values over NT
  public static final boolean TUNING_MODE = true;

  // the unit of measurement for Talon FX encoder velocity is known as the "Tran"
  // encoder ticks per 100ms
  public static double convertRPMToTrans(double RPM) {
    return RPM * 2048.0 / 600.0;
  }

  public static double convertTransToRPM(double trans) {
    return trans * 600.0 / 2048.0;
  }

  // used for unit conversions for ff constants on talon fx
  public static double voltageToPercent(double voltage) {
    return voltage / 12.0;
  }

  public static SupplyCurrentLimitConfiguration currentLimit(int fuseAmps) {
    return new SupplyCurrentLimitConfiguration(true, fuseAmps - 5, fuseAmps, 0.75);
  }

  public static final class kClimb {

    public static final int LEFT_MOTOR_CAN_ID = 2;
    public static final int RIGHT_MOTOR_CAN_ID = 17;

    public static final double OPEN_LOOP_UP_POWER = 0.3;
    public static final double OPEN_LOOP_DOWN_POWER = -1;

    public static final double LEFT_TOP_SETPOINT = -350000;
    public static final double RIGHT_TOP_SETPOINT = -350000;
    public static final double BOTTOM_SETPOINT = -10000;
    public static final double LATCH_PASSIVE = -42000;
    public static final double LATCH_MAIN = -300000;

    public static final double kP = 0.6;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;

    // timings in seconds for traversal
    public static final double MID_PASSIVE_LATCH_PAUSE = 3;
    public static final double HIGH_MAIN_LATCH_PAUSE = 3;
  }

  public static final class kOI {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // drive buttons
    public static final int INVERT_DRIVE = XboxController.Button.kY.value;

    // shooting buttons
    public static final int SHOOT = XboxController.Button.kB.value;
    public static final int RANGED_SHOOT = XboxController.Button.kLeftBumper.value;
    public static final int REVERSE_SHOOT = XboxController.Button.kBack.value;

    // intake buttons
    public static final int TOGGLE_INTAKE = XboxController.Button.kA.value;
    public static final int REVERSE_INTAKE = XboxController.Button.kStart.value;

    // climb buttons
    public static final int EXTEND_LATCH_MAIN = XboxController.Button.kX.value;
    public static final int CLIMB_LATCH_PASSIVE = XboxController.Button.kB.value;
    public static final int OPEN_LOOP_RAISE_CLIMB = XboxController.Button.kA.value;
    public static final int OPEN_LOOP_LOWER_CLIMB = XboxController.Button.kY.value;

    public static final String TRAJECTORY_NAME = "path";
  }

  public static final class kHopper {
    public static int MOTOR_ID;
    public static boolean INVERTED;
    public static final double SPEED = 0.7;
  }

  public static final class kIntake {
    public static final class kFalcon {
      public static final TalonFXInvertType INVERT_TYPE = TalonFXInvertType.CounterClockwise;
      public static final TalonFXControlMode CONTROL_MODE = TalonFXControlMode.PercentOutput;
      public static final int CURRENT_LIMIT = 30;
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
    public static int NUM_MOTORS = 4;

    // to divide quick turn power by
    public static final double QUICK_TURN_DAMPENER = 2.0;

    // char values for bear metal carpet
    public static final double ksVolts = 0.73137;
    public static final double kvVoltSecondsPerMeter = 2.388;
    public static final double kaVoltSecondsSquaredPerMeter = 0.52104;
    public static final double kPDriveVel = 0.00006707;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    // char values for royals carpet
    // public static final double ksVolts = 0.77377;
    // public static final double kvVoltSecondsPerMeter = 2.3111;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.23485;
    // public static final double kPDriveVel = 0.000020568;
    // public static final double kIDrive = 0;
    // public static final double kDDrive = 0;

    public static final double MAX_VOLTAGE = 5;

    // odometry constants - drivetrain measurements
    public static final double TRACK_WIDTH_METERS = 0.71; // width between sides of dt // 0.603
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    // ticks to meters conversion factor for falcon 500
    // (total ticks) * (motor rotations/tick) * (wheel rotations/motor rotations) *
    // (meters/wheel rotations)
    public static final double TICKS_TO_METERS = (1.0 / 2048.0) * (1.0 / 10.71) * (0.4788);

    // ramsete parameters
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double OPEN_LOOP_RAMP_RATE = 0; // 0.3
    public static final double QUICKTURN_DAMPENER = 3; // bigger number = slower turns
    public static final double SLOW_MODE_VELOCITY = -0.1;
    public static final int SUPPLY_LIMIT = 40;
    public static final int STATOR_LIMIT = 70;

    // slew constants
    public static final double TRIGGER_SPEED_DERIVATIVE = 0.04;
    public static final double LINEAR_SCALING_MIN_SPEED = 0.1;
    public static final double TRIGGER_SPEED_PROPORTIONAL = 0.03;
    public static final double MAX_ACCEL = 1;
    public static final double MINIMUM_SENSOR_VELOCITY = 0;

  }

  public static final class kShooter {

    public static final class kOpenLoop {
      public static final double SPEED = 1;
      public static final double BACK_SPEED = 1; // only used on double shooters
    }

    public static final class kDoubleClosedLoop {
      public static final double SETPOINT_RPM = 1325.0;// 1300 prac field // 1325 sundome // 1065 gpk CONSTANT

      public static final double FENDER_AMP = 1;
      public static final double FENDER_RATIO = 2.305;

      public static final double RANGED_AMP = 0.8;
      public static final double RANGED_RATIO = 5;

      public static final double AUTO_AMP = 1;
      public static final double AUTO_RATIO = 2.305;

      public static final class kFront {
        public static final double ERROR_TOLERANCE = 200; // 30
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
        public static final double ERROR_TOLERANCE = 200; // 30
        public static final double SETPOINT_OFFSET_RPM = 50;
        public static double kP;
        public static double kI;
        public static double kD;
        public static double kF;
        public static double kS;
        public static double kV;
        public static double kA;
        public static final double CURRENT_LIMIT = 15;
        public static final double CURRENT_LIMIT_THRESHOLD = 20;
        public static final double CURRENT_LIMIT_THRESHOLD_TIME = 3;
      }
    }

    public static final class kKicker {
      public static final double MOTOR_SPEED = 1;
      public static boolean KICKER_INVERSION;
    }

    public static int LEFT_MOTOR_ID;
    public static int RIGHT_MOTOR_ID;
    public static int KICKER_MOTOR_ID;
    public static int BACK_MOTOR_ID;
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
        kShooter.kKicker.KICKER_INVERSION = true;
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
        kShooter.kDoubleClosedLoop.kFront.kP = 0.1;
        kShooter.kDoubleClosedLoop.kFront.kI = 0.0000;
        kShooter.kDoubleClosedLoop.kFront.kD = 0.0;
        kShooter.kDoubleClosedLoop.kFront.kF = 0.05;
        kShooter.kDoubleClosedLoop.kBack.kP = 0.180;
        kShooter.kDoubleClosedLoop.kBack.kI = 0;
        kShooter.kDoubleClosedLoop.kBack.kD = 0;
        kShooter.kDoubleClosedLoop.kBack.kF = 0.045;
        kShooter.kDoubleClosedLoop.kBack.kS = voltageToPercent(0.0070982);
        kShooter.kDoubleClosedLoop.kBack.kV = voltageToPercent(0.0011253);
        kShooter.kDoubleClosedLoop.kBack.kA = voltageToPercent(0.000047908);
        kHopper.INVERTED = true;
        kShooter.kKicker.KICKER_INVERSION = false;
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
