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
  private final ProfiledPIDController pidController;
  private boolean closedLoop;
  private double goal;
  private double pidOutput;

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
    // right.setInverted(TalonFXInvertType.OppiseMaster);

    pidController = new ProfiledPIDController(
        Constants.kClimb.kP,
        Constants.kClimb.kI,
        Constants.kClimb.kD,
        new TrapezoidProfile.Constraints(
            Constants.kClimb.MAX_VELOCITY,
            Constants.kClimb.MAX_ACCELERATION));
    goal = 0;
  }

  public void zeroClimbEncoder() {
    left.setSelectedSensorPosition(0);
    left.set(ControlMode.PercentOutput, 0);
    // goal = 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("output", left.getMotorOutputPercent());
    SmartDashboard.putNumber("position", left.getSelectedSensorPosition());
    SmartDashboard.putNumber("goal", goal);
    /*
     * if (closedLoop) {
     * pidOutput = pidController.calculate(left.getSelectedSensorPosition(), goal);
     * left.set(pidOutput);
     * SmartDashboard.putNumber("pidOutput", pidOutput);
     * }
     */
  }

  @Override
  public void simulationPeriodic() {
  }

  public void setBrakeMode() {
    left.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    left.setNeutralMode(NeutralMode.Coast);
  }

  public void runClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
  }

  public void reverseClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
  }

  public void stopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, 0);
  }

  public void extendClimb() { // run command
    if (left.getSelectedSensorPosition() >= -165000) {
      left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
    } else {
      left.set(ControlMode.PercentOutput, 0);
    }
  }

  public void retractClimb() { // run command
    if (left.getSelectedSensorPosition() <= -3000) {
      left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
    } else {
      left.set(ControlMode.PercentOutput, 0);
    }
  }
}
