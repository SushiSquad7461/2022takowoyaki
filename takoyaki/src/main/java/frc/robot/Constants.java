// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
    public static final class kClimb {
        public static final int CLIMB_UP_BUTTON = XboxController.Button.kY.value;
        public static final int CLIMB_DOWN_BUTTON = XboxController.Button.kA.value;
        public static final int CLIMB_BRAKE_ON_BUTTON = XboxController.Button.kX.value;
        public static final int CLIMB_BRAKE_OFF_BUTTON = XboxController.Button.kB.value;

        public static final int climbSolenoidChannel = 1;
        public static final int LEFT_MOTOR_CAN_ID = 15;
        public static final int RIGHT_MOTOR_CAN_ID = 15;

        public static final double CLIMB_UP_MAX_POWER = 0.5;
        public static final double CLIMB_DOWN_MAX_POWER = -1;
        public static final int SOLENOID_FRONT = 1;
        public static final int SOLENOID_BACK = 0;
    }

    public static final class kOI {
        public static final int DRIVE_CONTROLLER = 0;
    }
}
