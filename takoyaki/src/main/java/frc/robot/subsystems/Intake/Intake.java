// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

  public abstract void runIntake();

  public abstract void stop();

  public abstract void reverseIntake();

  public abstract void runIntakeBackwards();

  public abstract void toggleIntake();

  public abstract void intakeShoot();

}
