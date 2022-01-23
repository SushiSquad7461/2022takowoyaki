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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private final CANSparkMax left = new CANSparkMax(Constants.kShooter.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax right = new CANSparkMax(Constants.kShooter.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = left.getEncoder();

  private final SimpleMotorFeedforward fForward;

  private double goal;

  public Shooter() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setIdleMode(CANSparkMax.IdleMode.kCoast);
    right.setIdleMode(CANSparkMax.IdleMode.kCoast);

    left.setInverted(false);
	
    // left.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kP, Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    // left.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kI, Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    // left.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kD, Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    right.follow(left, true);

    fForward = new SimpleMotorFeedforward(Constants.kShooter.kS, Constants.kShooter.kV);
  }

  public void run() {
    left.set(Constants.kShooter.SPEED);
    SmartDashboard.putBoolean("running shooter", true);
  }
  public void stop() {
    left.set(0);
    SmartDashboard.putBoolean("running shooter", false);
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
    //left.set(ControlMode.Velocity, goal, DemandType.ArbitraryFeedForward, fForward.calculate(goal));
    SmartDashboard.putNumber("spark faults", left.getStickyFaults());
    SmartDashboard.putNumber("spark rpm", leftEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() { }
}
