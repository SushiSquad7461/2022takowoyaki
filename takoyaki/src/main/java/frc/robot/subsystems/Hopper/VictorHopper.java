// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VictorHopper extends Hopper {
  private final WPI_VictorSPX floor = new WPI_VictorSPX(Constants.kHopper.MOTOR_ID);

  public VictorHopper() {
    floor.configFactoryDefault();
    floor.setInverted(Constants.kHopper.INVERTED);
    floor.setNeutralMode(NeutralMode.Brake);
  }

  public void runHopper() {
    SmartDashboard.putNumber("Hopper output", floor.getMotorOutputPercent());
    floor.set(ControlMode.PercentOutput,
        Constants.kHopper.SPEED * Math.abs(Math.sin(System.currentTimeMillis() / 10)));
  }

  public void reverseHopper() {
    floor.set(ControlMode.PercentOutput,
        -Constants.kHopper.SPEED * Math.abs(Math.sin(System.currentTimeMillis() / 1000)));
  }

  public void stop() {
    floor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
