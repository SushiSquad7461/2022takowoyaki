// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VictorHopper extends Hopper {
  private final WPI_TalonSRX floor = new WPI_TalonSRX(10);

  public VictorHopper() {
    floor.configFactoryDefault();
    floor.setInverted(!Constants.kHopper.INVERTED);
    floor.setNeutralMode(NeutralMode.Brake);
    floor.configSupplyCurrentLimit(Constants.currentLimit(30));
  }

  public void runHopper() {
    SmartDashboard.putNumber("Hopper output", floor.getMotorOutputPercent());
    // floor.set(ControlMode.PercentOutput,
    // Constants.kHopper.SPEED * Math.abs(Math.sin(System.currentTimeMillis() /
    // Constants.kHopper.JERKINESS)));
    floor.set(ControlMode.PercentOutput, 1);
  }

  public void reverseHopper() {
    floor.set(ControlMode.PercentOutput,
        -Constants.kHopper.SPEED);
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
