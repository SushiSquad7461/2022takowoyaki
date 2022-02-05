// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kShooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX left;
  private final WPI_TalonFX right;
  private final WPI_VictorSPX kicker;

  private final SimpleMotorFeedforward fForward;

  private double goal;

  public Shooter() {
    // instantiate motors
    left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
    right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
    kicker = new WPI_VictorSPX(Constants.kShooter.KICKER_MOTOR_ID);

    // set to brake mode on kicker, flywheel in coast
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
    kicker.setNeutralMode(NeutralMode.Brake);

    // inversion states of the shooter and kicker
    left.setInverted(kShooter.LEFT_INVERSION);
    right.setInverted(kShooter.RIGHT_INVERSION);
    kicker.setInverted(Constants.kShooter.KICKER_INVERSION);

    // configuring shooter PID values
    left.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kP,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kI,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kD,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    // set right motor to follow left
    right.follow(left);

    // configure feedforward
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

  public void reverseKicker() {
    kicker.set(ControlMode.PercentOutput, -kShooter.SPEED_KICKER);
  }

  public void setGoal(double goal) {
    this.goal = goal;
  }

  public void zeroGoal() {
    this.goal = 0;
  }

  public void setGoal() {
    this.goal = Constants.kShooter.GOAL;
  }

  @Override
  public void periodic() {
    left.set(ControlMode.Velocity, goal, DemandType.ArbitraryFeedForward, fForward.calculate(goal));
  }

  @Override
  public void simulationPeriodic() {
  }
}
