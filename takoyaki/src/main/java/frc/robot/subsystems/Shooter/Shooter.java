// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
  public abstract void runShooter();

  public abstract void stopShooter();

  public abstract void runKicker();

  public abstract void stopKicker();

  public abstract void reverseKicker();

  public abstract void zeroSetpoint();

  public abstract void setSetpoint();

  public abstract void setFenderSetpoint();

  public abstract void setRangedSetpoint();

  public abstract void setAutoSetpoint();

  public abstract boolean isAtSpeed();

  public abstract double getKickerOutput();

}
