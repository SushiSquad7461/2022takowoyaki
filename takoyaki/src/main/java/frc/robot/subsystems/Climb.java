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
  private final WPI_TalonFX right;
  private final ProfiledPIDController pidController;
  boolean closedLoop;

  public Climb() {

    closedLoop = false;

    left = new WPI_TalonFX(Constants.kClimb.LEFT_MOTOR_CAN_ID);
    right = new WPI_TalonFX(Constants.kClimb.RIGHT_MOTOR_CAN_ID);

    left.configFactoryDefault();
    right.configFactoryDefault();

    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);

    // right.follow(left);

    right.setInverted(TalonFXInvertType.Clockwise);
    left.setInverted(TalonFXInvertType.OpposeMaster);

    pidController = new ProfiledPIDController(
        Constants.kClimb.kP,
        Constants.kClimb.kI,
        Constants.kClimb.kD,
        new TrapezoidProfile.Constraints(
            Constants.kClimb.MAX_VELOCITY,
            Constants.kClimb.MAX_ACCELERATION));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("outptu", left.getMotorOutputPercent());
    if (closedLoop) {
      // left.set(pidController.calculate(left.getSelectedSensorPosition()));
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  // public void extendClimb() {
  // closedLoop = true;
  // pidController.setGoal(Constants.kClimb.TOP_LIMIT);
  // }

  // public void retractClimb() {
  // closedLoop = true;
  // pidController.setGoal(Constants.kClimb.BOTTOM_LIMIT);
  // }

  public void runClimb() {
    closedLoop = false;
    // fleft.setNeutralMode(NeutralMode.Coast);
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_UP_POWER);
    SmartDashboard.putBoolean("running", true);
  }

  public void reverseClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, Constants.kClimb.OPEN_LOOP_DOWN_POWER);
    SmartDashboard.putBoolean("running", true);
  }

  public void stopClimb() {
    closedLoop = false;
    left.set(ControlMode.PercentOutput, 0);
    // left.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putBoolean("running", false);
  }

  public void toggleOpenLoop() {
    closedLoop = !closedLoop;
  }
}
