// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OpenLoopFalconShooter extends Shooter {
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
  private final WPI_VictorSPX kicker = new WPI_VictorSPX(Constants.kShooter.KICKER_MOTOR_ID);

  double maxRPM;

  private double setpoint;

  public OpenLoopFalconShooter() {
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
    kicker.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.Clockwise);
    kicker.setInverted(Constants.kShooter.kKicker.KICKER_INVERSION);
    right.follow(left);

    this.zeroSetpoint();

    maxRPM = 0;
  }

  public void runShooter() {
    left.set(ControlMode.PercentOutput, Constants.kShooter.kOpenLoop.SPEED);
  }

  public void stopShooter() {
    left.set(ControlMode.PercentOutput, 0);
  }

  public void runKicker() {
    kicker.set(ControlMode.PercentOutput, Constants.kShooter.kKicker.MOTOR_SPEED);
  }

  public void stopKicker() {
    kicker.set(ControlMode.PercentOutput, 0);
  }

  public void reverseKicker() {
  }

  public void zeroSetpoint() {
    this.setpoint = 0;
  }

  public void setSetpoint() {
    this.setpoint = Constants.kShooter.kOpenLoop.SPEED;
  }

  public boolean isAtSpeed() {
    return false;
  }

  public void setRangedSetpoint() {

  }

  public double getKickerOutput() {
    return kicker.getMotorOutputVoltage();
  }

  @Override
  public void periodic() {
    if (left.getSelectedSensorVelocity() > maxRPM) {
      maxRPM = left.getSelectedSensorVelocity();
    }
    SmartDashboard.putNumber("maxrpm", maxRPM);
    SmartDashboard.putNumber("Current RMP", left.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Setpoint", setpoint);
    SmartDashboard.putNumber("applied output", left.getMotorOutputPercent());
    left.set(ControlMode.PercentOutput, setpoint);
    // left.set(ControlMode.PercentOutput, fForward.calculate(setpoint) / 12);
  }
}
