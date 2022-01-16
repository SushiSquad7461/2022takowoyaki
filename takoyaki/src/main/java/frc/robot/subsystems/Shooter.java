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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);

  private SimpleMotorFeedforward fForward;

  private double goal;

  public Shooter() {
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.Clockwise);
	
    left.config_kP(0, Constants.kShooter.kP, 100);
    left.config_kI(0, Constants.kShooter.kI, 100);
    left.config_kD(0, Constants.kShooter.kD, 100);

    right.follow(left);

    fForward = new SimpleMotorFeedforward(Constants.kShooter.kS, Constants.kShooter.kV);
  }

  public void run() {
    left.set(ControlMode.PercentOutput, Constants.kShooter.SPEED);
  }

  public void setGoal(double goal) {
    this.goal = goal;
  }

  public void setGoal() {
    this.goal = Constants.kShooter.GOAL;
  }

  @Override
  public void periodic() { 
    left.set(ControlMode.Velocity, goal, DemandType.ArbitraryFeedForward, fForward.calculate(goal));
  }

  @Override
  public void simulationPeriodic() { }
}
