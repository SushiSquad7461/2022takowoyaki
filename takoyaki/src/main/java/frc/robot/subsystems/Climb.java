// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Climb extends SubsystemBase {

  private final WPI_TalonFX left;
  private final WPI_TalonFX right;
  private boolean closedLoop;
  private boolean goingUp;

  public Climb() {

    closedLoop = false;

    left = new WPI_TalonFX(Constants.kClimb.LEFT_MOTOR_CAN_ID);
    right = new WPI_TalonFX(Constants.kClimb.RIGHT_MOTOR_CAN_ID);

    motorConfig(left);
    motorConfig(right);

    // right.follow(left);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.OpposeMaster);
  }

  public void zeroClimbEncoder() {
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("output", left.getMotorOutputPercent());
    SmartDashboard.putNumber("position", left.getSelectedSensorPosition());

  }

  public void defaultCommand(double openLoopLeft, double openLoopRight) {
    if (!closedLoop) {
      if (openLoopLeft > 0.5) {
        left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
      } else if (openLoopLeft < -0.5) {
        left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
      } else {
        left.set(ControlMode.PercentOutput, 0);
      }

      if (openLoopRight > 0.5) {
        right.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
      } else if (openLoopLeft < -0.5) {
        right.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
      } else {
        right.set(ControlMode.PercentOutput, 0);
      }
    } else {
      if (goingUp) {
        if (left.getSelectedSensorPosition() <= Constants.kClimb.TOP_ENCODER_VAL) {
          left.set(ControlMode.PercentOutput, 0);
        }
        if (right.getSelectedSensorPosition() <= Constants.kClimb.TOP_ENCODER_VAL) {
          left.set(ControlMode.PercentOutput, 0);
        }
        if (left.getSelectedSensorPosition() <= Constants.kClimb.TOP_ENCODER_VAL
            && right.getSelectedSensorPosition() <= Constants.kClimb.TOP_ENCODER_VAL) {
          closedLoop = false;
        }
      } else {
        if (left.getSelectedSensorPosition() >= Constants.kClimb.BOTTOM_ENCODER_VAL) {
          left.set(ControlMode.PercentOutput, 0);
        }
        if (right.getSelectedSensorPosition() >= Constants.kClimb.BOTTOM_ENCODER_VAL) {
          left.set(ControlMode.PercentOutput, 0);
        }
        if (left.getSelectedSensorPosition() >= Constants.kClimb.TOP_ENCODER_VAL
            && right.getSelectedSensorPosition() >= Constants.kClimb.TOP_ENCODER_VAL) {
          closedLoop = false;
        }
      }
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  public void stopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, 0);
  }

  public void extendClimb() {
    closedLoop = true;
    goingUp = true;
    left.set(ControlMode.PercentOutput, Constants.kClimb.CLOSED_LOOP_UP_POWER);
    right.set(ControlMode.PercentOutput, Constants.kClimb.CLOSED_LOOP_UP_POWER);
  }

  public void retractClimb() {
    closedLoop = true;
    goingUp = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.CLOSED_LOOP_DOWN_POWER);
    right.set(ControlMode.PercentOutput, Constants.kClimb.CLOSED_LOOP_DOWN_POWER);
  }

  public void motorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(Constants.kClimb.OPEN_LOOP_RAMP_RATE);

  }
}
