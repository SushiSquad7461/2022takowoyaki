// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
  private final WPI_VictorSPX kicker = new WPI_VictorSPX(Constants.kShooter.KICKER_MOTOR_ID);
  
  private final DigitalInput shooterSensor;

  private final SimpleMotorFeedforward fForward;

  private double goal;

  public Shooter() {
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

    fForward = new SimpleMotorFeedforward(Constants.kShooter.kS, Constants.kShooter.kV);
    shooterSensor = new DigitalInput(Constants.kShooter.SHOOTER_SENSOR);

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

  public void setGoal(double goal) {
    this.goal = goal;
  }

  public void zeroGoal() {
    this.goal = 0;
  }

  public void setGoal() {
    this.goal = Constants.kShooter.GOAL;
  }

  public boolean isAtSpeed() {
    if((left.getSelectedSensorVelocity() * 2048.0 / 600.0) >= (Constants.kShooter.GOAL * 0.9)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    if(goal == 0) {
      left.set(ControlMode.PercentOutput, 0);
    } else {
      left.set(ControlMode.Velocity, goal);
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  public boolean beamBroken() {
    SmartDashboard.putBoolean("Beam Broken", !shooterSensor.get());
    return !shooterSensor.get();
  }
}
