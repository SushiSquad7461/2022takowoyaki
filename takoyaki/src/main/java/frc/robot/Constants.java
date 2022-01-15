// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
    public static int
        climbUp = XboxController.Button.kY.value,
        climbDown = XboxController.Button.kA.value,
        climbBrakeOn = XboxController.Button.kX.value,
        climbBrakeOff = XboxController.Button.kB.value,
        
        climbSolenoidChannel = 1,
        climbLeftCAN = 15,
        climbRightCAN = 15
    ;
    public static double
        climbMaxUp = 0.5,
        climbMaxDown = 1
    ;
}
