// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climb extends SubsystemBase {

    WPI_TalonFX left = new WPI_TalonFX(15);
    WPI_TalonFX right = new WPI_TalonFX(16);

  public Climb() {
      
      left.follow(right);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
  }
}
