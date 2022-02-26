// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClosedLoopFalconComp extends Shooter {
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
  private final WPI_TalonSRX kicker = new WPI_TalonSRX(Constants.kShooter.KICKER_MOTOR_ID);

  private final SimpleMotorFeedforward fForward;

  private double change;

  private double setpoint;

  public ClosedLoopFalconComp() {
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
    kicker.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.Clockwise);
    kicker.setInverted(Constants.kShooter.KICKER_INVERSION);

    left.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kP,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kI,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kD,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kF(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kF,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    right.follow(left);

    fForward = new SimpleMotorFeedforward(Constants.kShooter.kS, Constants.kShooter.kV, Constants.kShooter.kA);

    this.zeroSetpoint();
  }

  public void runShooter() {
    left.set(ControlMode.PercentOutput, Constants.kShooter.SPEED);
  }

  public void stopShooter() {
    left.set(ControlMode.PercentOutput, 0);
  }

  public void runKicker() {
    kicker.set(ControlMode.PercentOutput, Constants.kShooter.SPEED_KICKER);
  }

  public void stopKicker() {
    kicker.set(ControlMode.PercentOutput, 0);
  }

  public void reverseKicker() {
    kicker.set(ControlMode.PercentOutput, -Constants.kShooter.SPEED_KICKER);
  }

  @Override
  public void setSetpoint(double setpoint) {
    SmartDashboard.putBoolean("Setpoint set", true);
    this.setpoint = setpoint;
  }

  public void zeroSetpoint() {
    SmartDashboard.putBoolean("Setpoint set", false);
    this.setpoint = 0;
  }

  public void setSetpoint() {
    this.setpoint = Constants.kShooter.kClosedLoop.SETPOINT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current encoder ticks per 100 ms", left.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Current rpm", left.getSelectedSensorVelocity() * 600.0 / 2048.0);
    SmartDashboard.putNumber("Setpoint", setpoint);
    SmartDashboard.putNumber("applied output", left.getMotorOutputPercent());
    SmartDashboard.putNumber("feed forward", fForward.calculate(setpoint));
    SmartDashboard.putNumber("shooter position change", left.getSelectedSensorPosition() - change);
    SmartDashboard.putBoolean("at speed", isAtSpeed());
    change = left.getSelectedSensorPosition();

    if (setpoint == 0) {
      left.set(ControlMode.PercentOutput, 0);
    } else {
      left.set(ControlMode.Velocity, setpoint);
    }
    // left.set(ControlMode.PercentOutput, fForward.calculate(setpoint) / 12);
  }

  public boolean isAtSpeed() {
    double difference = Math.abs(left.getSelectedSensorVelocity() - Constants.kShooter.kClosedLoop.SETPOINT);
    if (difference <= Constants.kShooter.kClosedLoop.ERROR_TOLERANCE) return true;
    else return false;
  }

  @Override
  public void simulationPeriodic() {
  }
}
