// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ClosedLoopFalconShooter extends Shooter {
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
  private final WPI_VictorSPX kicker = new WPI_VictorSPX(Constants.kShooter.KICKER_MOTOR_ID);

  private final SimpleMotorFeedforward fForward;

  private double setpoint;

  public ClosedLoopFalconShooter() {
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

    right.follow(left);

    fForward = new SimpleMotorFeedforward(Constants.kShooter.kS, Constants.kShooter.kV);
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

  @Override
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void zeroSetpoint() {
    this.setpoint = 0;
  }

  public void setSetpoint() {
    this.setpoint = Constants.kShooter.SETPOINT;
  }

  @Override
  public void periodic() {
    left.set(ControlMode.Velocity, setpoint, DemandType.ArbitraryFeedForward, fForward.calculate(setpoint));
  }

  @Override
  public void simulationPeriodic() {
  }
}
