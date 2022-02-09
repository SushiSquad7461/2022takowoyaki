// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;

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
  }

  public static final class kHopper {
    public static final int MOTOR_ID = 10;
    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 30;
    public static final double SPEED = 0.9;

    public static final double OPEN_LOOP_RAMP_RATE = 0;
  }

  public static final class kIntake {
    public static final class kFalcon {
      public static final TalonFXInvertType INVERT_TYPE = TalonFXInvertType.Clockwise;
      public static final TalonFXControlMode CONTROL_MODE = TalonFXControlMode.PercentOutput;
    }

    public static final class kSpark {
      public static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
      public static final int CURRENT_LIMIT = 35;
      public static final int OPEN_LOOP_RAMP_RATE = 0;
      public static final boolean INVERTED = true;
    }

    public static final int MOTOR_ID = 8;
    public static final int SOLENOID_FRONT = 1;
    public static final int SOLENOID_BACK = 0;
    public static final double INTAKE_SPEED = 0.9;
  }

  public static final class kDrive {
    public static final int FRONT_RIGHT_ID = 3;
    public static final int FRONT_LEFT_ID = 1;
    public static final int BACK_RIGHT_ID = 4;
    public static final int BACK_LEFT_ID = 2;
  }

  public static final class kShooter {
    public static final class kOpenLoop {
      public static final double SETPOINT = 1;
    }

    public static final class kClosedLoop {
      public static final double SETPOINT = 3700.0 * 2048.0 / 600.0;
    }

    public static final int LEFT_MOTOR_ID = 12;
    public static final int RIGHT_MOTOR_ID = 15;
    public static final int KICKER_MOTOR_ID = 5;
    public static final double SPEED = 0.9;
    public static final int CURRENT_LIMIT = 35;
    public static final double kP = 0.07;
    public static final double kI = 0.00001;
    public static final double kD = 0.0;
    public static final double kF = 0.045;
    public static final double kS = 0.61716 / 12.0;
    public static final double kV = 0.10724 / 12.0;
    public static final double kA = 0.0082862 / 12.0;
    public static final int DEFAULT_PROFILE_SLOT = 0;
    public static final int DEFAULT_CONFIG_TIMEOUT = 100;

    public static final boolean KICKER_INVERSION = true;

    public static final double SPEED_KICKER = 1;
  }
}
