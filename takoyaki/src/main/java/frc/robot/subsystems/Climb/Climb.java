// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public abstract class Climb extends SubsystemBase {

  public abstract void zeroClimbEncoders();

  public abstract void defaultCommand(double leftTrigger, double rightTrigger);

  public abstract void stopClimb();

  public abstract void extendClimb();

  public abstract void retractClimb();

  public abstract void rejoinClimb();

  public abstract void separateClimb();

  public abstract void runClimb();

  public abstract void climbDown();
}
