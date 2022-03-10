// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClosedLoopSingleFalconShooter extends Shooter {
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
  private final WPI_TalonSRX kicker = new WPI_TalonSRX(Constants.kShooter.KICKER_MOTOR_ID);

  private final SimpleMotorFeedforward fForward;

  private double change;

  private double setpoint;

  public ClosedLoopSingleFalconShooter() {
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
    kicker.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.Clockwise);
    kicker.setInverted(Constants.kShooter.kKicker.KICKER_INVERSION);

    left.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kSingleClosedLoop.kP,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kSingleClosedLoop.kI,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kSingleClosedLoop.kD,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kF(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kSingleClosedLoop.kF,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    right.follow(left);

    fForward = new SimpleMotorFeedforward(Constants.kShooter.kSingleClosedLoop.kS,
        Constants.kShooter.kSingleClosedLoop.kV, Constants.kShooter.kSingleClosedLoop.kA);

    this.zeroSetpoint();
  }

  public void runShooter() {
    left.set(ControlMode.PercentOutput, Constants.kShooter.kOpenLoop.SPEED);
  }

  public void stopShooter() {
    left.set(ControlMode.PercentOutput, 0);
  }

  public void runKicker() {
    kicker.set(ControlMode.PercentOutput, Constants.kShooter.kKicker.MOTOR_SPEED);
    // double kickerOutput = Math.round(Math.sin(System.currentTimeMillis() /
    // Constants.kShooter.KICKER_PERIOD) + Constants.kShooter.KICKER_OFFSET);
    // if (kickerOutput <= 0) kicker.set(ControlMode.PercentOutput, 0);
    // else kicker.set(ControlMode.PercentOutput, kickerOutput);
  }

  public void stopKicker() {
    kicker.set(ControlMode.PercentOutput, 0);
  }

  public void reverseKicker() {
    kicker.set(ControlMode.PercentOutput, -Constants.kShooter.kKicker.MOTOR_SPEED);
  }

  public void zeroSetpoint() {
    SmartDashboard.putBoolean("Setpoint set", false);
    this.setpoint = 0;
  }

  public void setSetpoint() {
    this.setpoint = Constants.kShooter.kSingleClosedLoop.SETPOINT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current encoder ticks per 100 ms", left.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Current rpm", left.getSelectedSensorVelocity() * 600.0 / 2048.0);
    SmartDashboard.putNumber("Setpoint", setpoint);
    SmartDashboard.putBoolean("at speed", isAtSpeed());
    SmartDashboard.putNumber("kicker speed", kicker.getMotorOutputPercent());
    change = left.getSelectedSensorPosition();

    if (setpoint == 0) {
      left.set(ControlMode.PercentOutput, 0);
    } else {
      left.set(ControlMode.Velocity, setpoint);
    }
    // left.set(ControlMode.PercentOutput, fForward.calculate(setpoint) / 12);
  }

  public boolean isAtSpeed() {
    double difference = Math
        .abs(left.getSelectedSensorVelocity() - Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT);
    if (difference <= Constants.kShooter.kDoubleClosedLoop.kFront.ERROR_TOLERANCE)
      return true;
    else
      return false;
  }

  @Override
  public void simulationPeriodic() {
  }
}
