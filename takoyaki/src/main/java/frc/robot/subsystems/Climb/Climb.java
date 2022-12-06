// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climb extends SubsystemBase {
  public abstract void runClimb();

  public abstract void reverseClimb();

  public abstract void stopClimb();

  public abstract void extendClimb();

  public abstract void retractClimb();

  public abstract void latchPassive();

  public abstract void latchMain();

  public abstract boolean isFinished();
}
