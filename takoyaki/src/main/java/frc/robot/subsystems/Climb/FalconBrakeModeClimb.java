// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
  private double setpoint;
  private boolean closedLoop;

  public FalconBrakeModeClimb() {
    left = new WPI_TalonFX(Constants.kClimb.LEFT_MOTOR_CAN_ID);
    right = new WPI_TalonFX(Constants.kClimb.RIGHT_MOTOR_CAN_ID);

    left.config_kP(0, Constants.kClimb.kP);
    left.config_kI(0, Constants.kClimb.kI);
    left.config_kD(0, Constants.kClimb.kD);
    left.config_kF(0, Constants.kClimb.kF);

    motorConfig(left);
    motorConfig(right);

    right.follow(left);

    left.setInverted(TalonFXInvertType.Clockwise);
    right.setInverted(TalonFXInvertType.OpposeMaster);

    closedLoop = true;
    setpoint = Constants.kClimb.BOTTOM_SETPOINT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("output", left.getMotorOutputPercent());
    SmartDashboard.putNumber("position", left.getSelectedSensorPosition());

    if (closedLoop) {
      left.set(ControlMode.Position, setpoint);
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  private void motorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(Constants.kClimb.OPEN_LOOP_RAMP_RATE);
    motor.configSupplyCurrentLimit(Constants.currentLimit(40));

  }

  public double getEncoder() {
    return left.getSelectedSensorPosition();
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void extendClimb() {
    this.setpoint = Constants.kClimb.TOP_SETPOINT;
  }

  public void retractClimb() {
    this.setpoint = Constants.kClimb.BOTTOM_SETPOINT;
  }

  public void stopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, 0);
  }

  public void runClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
  }

  public void reverseClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
  }

  public void releasePassiveHook() {
    this.setSetpoint(Constants.kClimb.TOP_SETPOINT - Constants.kClimb.UNHOOK_DISTANCE);
  }

  public void zeroClimbEncoders() {
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }
}
