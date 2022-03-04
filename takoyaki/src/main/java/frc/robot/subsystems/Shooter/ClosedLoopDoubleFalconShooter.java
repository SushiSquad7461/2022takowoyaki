// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClosedLoopDoubleFalconShooter extends Shooter {
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private final WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);
  private final WPI_TalonFX back = new WPI_TalonFX(Constants.kShooter.BACK_MOTOR_ID);
  private final WPI_TalonSRX kicker = new WPI_TalonSRX(Constants.kShooter.KICKER_MOTOR_ID);

  private final SimpleMotorFeedforward frontFeedForward;
  private final SimpleMotorFeedforward backFeedForward;

  private double frontChange;
  private double backChange;
  private double frontSetpoint;
  private double backSetpoint;

  public ClosedLoopDoubleFalconShooter() {
    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
    back.setNeutralMode(NeutralMode.Coast);
    kicker.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.Clockwise);
    back.setInverted(TalonFXInvertType.Clockwise);
    kicker.setInverted(!Constants.kShooter.KICKER_INVERSION);

    left.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kFront.kP,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kFront.kI,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kFront.kD,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kF(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kFront.kF,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    back.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kBack.kP,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    back.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kBack.kI,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    back.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kBack.kD,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    back.config_kF(Constants.kShooter.DEFAULT_PROFILE_SLOT, Constants.kShooter.kDoubleClosedLoop.kBack.kF,
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);


    right.follow(left);

    frontFeedForward = new SimpleMotorFeedforward(Constants.kShooter.kDoubleClosedLoop.kFront.kS,
        Constants.kShooter.kDoubleClosedLoop.kFront.kV, Constants.kShooter.kDoubleClosedLoop.kFront.kA);

    backFeedForward = new SimpleMotorFeedforward(Constants.kShooter.kDoubleClosedLoop.kBack.kS,
        Constants.kShooter.kDoubleClosedLoop.kBack.kV, Constants.kShooter.kDoubleClosedLoop.kBack.kA);

    this.zeroSetpoint();
  }

  public void runShooter() {
    left.set(ControlMode.PercentOutput, Constants.kShooter.kOpenLoop.SPEED);
    back.set(ControlMode.PercentOutput, Constants.kShooter.kOpenLoop.BACK_SPEED);
  }

  public void stopShooter() {
    left.set(ControlMode.PercentOutput, 0);
    back.set(ControlMode.PercentOutput, 0);
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

  public void zeroSetpoint() {
    SmartDashboard.putBoolean("Setpoint set", false);
    this.frontSetpoint = 0;
    this.backSetpoint = 0;
  }

  public void setSetpoint() {
    this.frontSetpoint = Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT + Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET;
    this.backSetpoint = Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT + Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET;
  }

  @Override
  public void periodic() {
    // runKicker();
    SmartDashboard.putNumber("front shooter rpm", left.getSelectedSensorVelocity() * 600.0 / 2048.0);
    SmartDashboard.putNumber("front shooter setpoint", (frontSetpoint- Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET) * 600.0 / 2048.0);
    SmartDashboard.putNumber("back shooter setpoint", (backSetpoint- Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET) * 600.0 / 2048.0);
    SmartDashboard.putNumber("back shooter rpm", back.getSelectedSensorVelocity() * 600.0 / 2048.0);
    frontChange = left.getSelectedSensorPosition();
    //back.set(ControlMode.PercentOutput, Constants.kShooter.kOpenLoop.BACK_SPEED);
    
    if (frontSetpoint == 0) { // assume both setpoints are zero
      left.set(ControlMode.PercentOutput, 0);
      back.set(ControlMode.PercentOutput, 0);
    } else {
      left.set(ControlMode.Velocity, frontSetpoint);
      back.set(ControlMode.Velocity, backSetpoint);
    }
    // left.set(ControlMode.PercentOutput, fForward.calculate(setpoint) / 12);
  }

  public boolean isAtSpeed() {
    if (((Constants.convertRPMtoTrans(
        left.getSelectedSensorVelocity()) >= Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT * 0.9)
        &&
        (Constants.convertRPMtoTrans(
            back.getSelectedSensorVelocity()) >= Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT * 0.9))) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
