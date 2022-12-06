// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

  // run only the motor
  public abstract void runIntakeMotor();

  // run only the motor in reverse
  public abstract void runIntakeMotorBackwards();

  // extend and intake
  public abstract void intake();

  // retract and stop
  public abstract void stop();

  // extend and reverse
  public abstract void outtake();

  // toggle the intake state (extended or stopped)
  public abstract void toggleIntake();
}
