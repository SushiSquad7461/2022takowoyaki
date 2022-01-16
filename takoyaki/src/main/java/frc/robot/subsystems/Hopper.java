// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  WPI_TalonSRX floor = new WPI_TalonSRX(Constants.kHooper.MOTOR_ID);

  public Hopper() {
    floor.configFactoryDefault();
    floor.setInverted(Constants.kHooper.INVERTED);
    floor.setNeutralMode(NeutralMode.Coast);
    floor.configPeakCurrentLimit(Constants.kHooper.CURRENT_LIMIT);
    floor.enableCurrentLimit(true);
  }

  public void runForward() {
    floor.set(ControlMode.PercentOutput, Constants.kHooper.SPEED);
  }

  public void runBackward() {
    floor.set(ControlMode.PercentOutput, -Constants.kHooper.SPEED);
  }

  public void stop() {
    floor.set(ControlMode.PercentOutput, 0);
  }


  @Override 
  public void periodic() { }

  @Override 
  public void simulationPeriodic() { }
}
