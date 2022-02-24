// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Climb extends SubsystemBase {

  private final WPI_TalonFX left;
  // private final WPI_TalonFX right;
  private boolean closedLoop;
  private boolean goingUp;

  public Climb() {

    closedLoop = false;

    left = new WPI_TalonFX(Constants.kClimb.LEFT_MOTOR_CAN_ID);
    // right = new WPI_TalonFX(Constants.kClimb.RIGHT_MOTOR_CAN_ID);

    left.configFactoryDefault();
    // right.configFactoryDefault();

    left.setSelectedSensorPosition(0);
    // right.setSelectedSensorPosition(0);

    left.setNeutralMode(NeutralMode.Brake);
    // right.setNeutralMode(NeutralMode.Brake);

    // right.follow(left);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    // right.setInverted(TalonFXInvertType.OpposeMaster);

    left.configOpenloopRamp(kClimb.OPEN_LOOP_RAMP_RATE);
    // right.configOpenloopRamp(kClimb.OPEN_LOOP_RAMP_RATE);
  }

  public void zeroClimbEncoder() {
    left.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("output", left.getMotorOutputPercent());
    SmartDashboard.putNumber("position", left.getSelectedSensorPosition());
    
    if (closedLoop && goingUp && left.getSelectedSensorPosition() >= kClimb.TOP_ENCODER_VAL) {
      // Going Up
      left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
    } else if (closedLoop && !goingUp && left.getSelectedSensorPosition() <= kClimb.BOTTOM_ENCODER_VAL) {
      // Going down
      left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
    } else if (closedLoop) {
      // Stop
      closedLoop = false;
      left.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void simulationPeriodic() { }

  public void runOpenLoopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
  }

  public void reverseOpenLoopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
  }

  public void stopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, 0);
  }

  public void toggleExtendClimb() {
    if (!closedLoop) {
      closedLoop = true;
      goingUp = true;
    } else if (closedLoop && !goingUp) {
      goingUp = true;
    } else {
      stopClimb();
    }
  }

  public void toggleRetractClimb() {
    if (!closedLoop) {
      closedLoop = true;
      goingUp = false;
    } else if (closedLoop && goingUp) {
      goingUp = false;
    } else {
      stopClimb();
    }
  }
}
