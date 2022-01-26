// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private final WPI_VictorSPX floor = new WPI_VictorSPX(Constants.kHopper.MOTOR_ID);
  private final WPI_VictorSPX kicker = new WPI_VictorSPX(Constants.kHopper.KICKER_ID);

  public Hopper() {
    floor.configFactoryDefault();
    floor.setInverted(Constants.kHopper.INVERTED);
    floor.setNeutralMode(NeutralMode.Coast);

    kicker.configFactoryDefault();
    kicker.setInverted(!Constants.kHopper.INVERTED);
    kicker.setNeutralMode(NeutralMode.Coast);
    // floor.configPeakCurrentLimit(Constants.kHopper.CURRENT_LIMIT);
    // floor.enableCurrentLimit(true);
  }

  public void runForward() {
    floor.set(ControlMode.PercentOutput, Constants.kHopper.SPEED);
    kicker.set(ControlMode.PercentOutput, 1);
    SmartDashboard.putString("Running hopper", "forward");
  }

  public void runBackward() {
    floor.set(ControlMode.PercentOutput, -Constants.kHopper.SPEED);
    SmartDashboard.putString("Running hopper", "backward");
  }

  public void stop() {
    floor.set(ControlMode.PercentOutput, 0);
    kicker.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putString("Running hopper", "stop");
  }

  @Override 
  public void periodic() { 
    floor.set(ControlMode.PercentOutput, Constants.kHopper.SPEED);
  }

  @Override 
  public void simulationPeriodic() { }
}