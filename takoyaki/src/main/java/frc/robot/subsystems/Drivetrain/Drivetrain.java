// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase {
  public abstract void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn);
  public abstract void setToBrakeMode();
  public abstract void setToCoastMode();
  public abstract void invertDrive();
}