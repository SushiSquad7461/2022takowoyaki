// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import javax.imageio.spi.RegisterableService;
import javax.naming.ldap.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class FalconBrakeModeClimb extends Climb {

  private final WPI_TalonFX left;
  private final WPI_TalonFX right;
  private double leftSetpoint;
  private double rightSetpoint;
  private boolean closedLoop;

  public FalconBrakeModeClimb() {
    left = new WPI_TalonFX(Constants.kClimb.LEFT_MOTOR_CAN_ID);
    right = new WPI_TalonFX(Constants.kClimb.RIGHT_MOTOR_CAN_ID);

    configMotor(left);
    configMotor(right);

    left.config_kP(0, Constants.kClimb.kP);
    left.config_kI(0, Constants.kClimb.kI);
    left.config_kD(0, Constants.kClimb.kD);
    left.config_kF(0, Constants.kClimb.kF);

    right.config_kP(0, Constants.kClimb.kP);
    right.config_kI(0, Constants.kClimb.kI);
    right.config_kD(0, Constants.kClimb.kD);
    right.config_kF(0, Constants.kClimb.kF);

    left.setInverted(TalonFXInvertType.Clockwise);
    right.setInverted(TalonFXInvertType.CounterClockwise);

    closedLoop = false;
    zeroClimbEncoders();

    left.configForwardSoftLimitEnable(true);
    left.configReverseSoftLimitEnable(true);
    left.configForwardSoftLimitThreshold(Constants.kClimb.LEFT_MAX_HEIGHT);
    left.configReverseSoftLimitThreshold(Constants.kClimb.LEFT_MIN_HEIGHT);
    right.configForwardSoftLimitEnable(true);
    right.configReverseSoftLimitEnable(true);
    right.configForwardSoftLimitThreshold(Constants.kClimb.RIGHT_MAX_HEIGHT);
    right.configReverseSoftLimitThreshold(Constants.kClimb.RIGHT_MIN_HEIGHT);

    leftSetpoint = Constants.kClimb.BOTTOM_SETPOINT;
    rightSetpoint = Constants.kClimb.BOTTOM_SETPOINT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left position", left.getSelectedSensorPosition());
    SmartDashboard.putNumber("right position", right.getSelectedSensorPosition());
    SmartDashboard.putNumber("left setpoint", leftSetpoint);
    SmartDashboard.putNumber("right setpoint", rightSetpoint);
    if (closedLoop) {
      left.set(ControlMode.Position, leftSetpoint);
      right.set(ControlMode.Position, rightSetpoint);
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  private void configMotor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSupplyCurrentLimit(Constants.currentLimit(40));
  }

  public boolean isFinished() {
    if (leftSetpoint == Constants.kClimb.LEFT_TOP_SETPOINT) {
      return left.getSelectedSensorPosition() <= leftSetpoint && right.getSelectedSensorPosition() <= rightSetpoint;
    } else {
      return left.getSelectedSensorPosition() >= leftSetpoint && right.getSelectedSensorPosition() >= rightSetpoint;
    }
  }

  private void setSetpoint(double left, double right) {
    closedLoop = true;
    this.leftSetpoint = left;
    this.rightSetpoint = right;
  }

  private void setSetpoint(double setpoint) {
    closedLoop = true;
    this.leftSetpoint = setpoint;
    this.rightSetpoint = setpoint;
  }

  public void setPower(double leftPower, double rightPower) {
    closedLoop = false;
    left.set(leftPower);
    right.set(rightPower);
  }

  public void extendClimb() {
    setSetpoint(Constants.kClimb.LEFT_TOP_SETPOINT, Constants.kClimb.RIGHT_TOP_SETPOINT);
  }

  public void retractClimb() {
    setSetpoint(Constants.kClimb.BOTTOM_SETPOINT);
  }

  public void latchPassive() {
    setSetpoint(Constants.kClimb.LATCH_PASSIVE);
  }

  public void latchMain() {
    setSetpoint(Constants.kClimb.LATCH_MAIN);
  }

  public void stopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, 0);
    right.set(ControlMode.PercentOutput, 0);
  }

  public void runClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
    right.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
  }

  public void reverseClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
    right.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
  }

  public void calibrationMode() {
    left.configForwardSoftLimitEnable(false);
    left.configReverseSoftLimitEnable(false);

    
    right.configForwardSoftLimitEnable(false);
    right.configReverseSoftLimitEnable(false);
  }

  public void zeroClimbEncoders() {
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }

}
