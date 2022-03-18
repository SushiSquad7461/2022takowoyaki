// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.utils.TunableNumber;

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
  private double frontSetpointRPMWithOffset;
  private double backSetpointRPMWithOffset;
  private boolean tunable;
  private TunableNumber frontSetpointRPM = new TunableNumber("front shooter rpm goal",
      Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_RPM);
  private TunableNumber backSetpointRPM = new TunableNumber("back shooter rpm goal",
      Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_RPM);

  private TunableNumber frontP = new TunableNumber("frontP", Constants.kShooter.kDoubleClosedLoop.kFront.kP);
  private TunableNumber frontI = new TunableNumber("frontI", Constants.kShooter.kDoubleClosedLoop.kFront.kI);
  private TunableNumber frontD = new TunableNumber("frontD", Constants.kShooter.kDoubleClosedLoop.kFront.kD);
  private TunableNumber frontF = new TunableNumber("frontF", Constants.kShooter.kDoubleClosedLoop.kFront.kF);

  private TunableNumber backP = new TunableNumber("frontP", Constants.kShooter.kDoubleClosedLoop.kFront.kP);
  private TunableNumber backI = new TunableNumber("frontI", Constants.kShooter.kDoubleClosedLoop.kFront.kI);
  private TunableNumber backD = new TunableNumber("frontD", Constants.kShooter.kDoubleClosedLoop.kFront.kD);
  private TunableNumber backF = new TunableNumber("frontF", Constants.kShooter.kDoubleClosedLoop.kFront.kF);

  public ClosedLoopDoubleFalconShooter() {

    left.setNeutralMode(NeutralMode.Coast);
    right.setNeutralMode(NeutralMode.Coast);
    back.setNeutralMode(NeutralMode.Coast);
    kicker.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.CounterClockwise);
    right.setInverted(TalonFXInvertType.Clockwise);
    back.setInverted(TalonFXInvertType.Clockwise);
    kicker.setInverted(!Constants.kShooter.KICKER_INVERSION);

    left.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, frontP.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, frontI.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, frontD.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    left.config_kF(Constants.kShooter.DEFAULT_PROFILE_SLOT, frontF.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    back.config_kP(Constants.kShooter.DEFAULT_PROFILE_SLOT, backP.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    back.config_kI(Constants.kShooter.DEFAULT_PROFILE_SLOT, backI.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    back.config_kD(Constants.kShooter.DEFAULT_PROFILE_SLOT, backD.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);
    back.config_kF(Constants.kShooter.DEFAULT_PROFILE_SLOT, backF.get(),
        Constants.kShooter.DEFAULT_CONFIG_TIMEOUT);

    right.follow(left);

    frontFeedForward = new SimpleMotorFeedforward(Constants.kShooter.kDoubleClosedLoop.kFront.kS,
        Constants.kShooter.kDoubleClosedLoop.kFront.kV, Constants.kShooter.kDoubleClosedLoop.kFront.kA);

    backFeedForward = new SimpleMotorFeedforward(Constants.kShooter.kDoubleClosedLoop.kBack.kS,
        Constants.kShooter.kDoubleClosedLoop.kBack.kV, Constants.kShooter.kDoubleClosedLoop.kBack.kA);

    tunable = false;

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
    this.frontSetpointRPMWithOffset = 0;
    this.backSetpointRPMWithOffset = 0;
  }

  public void setSetpoint() {
    if (tunable) {
      this.frontSetpointRPMWithOffset = frontSetpointRPM.get()
          + Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET_RPM;
      this.backSetpointRPMWithOffset = backSetpointRPM.get()
          + Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET_RPM;
    } else {
      this.frontSetpointRPMWithOffset = Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_RPM
          + Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET_RPM;
      this.backSetpointRPMWithOffset = Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_RPM
          + Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET_RPM;
    }
  }

  public void setRangedSetpoint() {
    frontSetpointRPMWithOffset = Constants.kShooter.kDoubleClosedLoop.kFront.RANGED_SETPOINT
        + Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET_RPM;
    backSetpointRPMWithOffset = Constants.kShooter.kDoubleClosedLoop.kBack.RANGED_SETPOINT
        + Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET_RPM;
  }

  @Override
  public void periodic() {
    // runKicker();
    SmartDashboard.putNumber("front shooter actual rpm", left.getSelectedSensorVelocity() * 600.0 / 2048.0);
    // SmartDashboard.putNumber("front shooter setpoint",
    // (frontSetpointRPMWithOffset -
    // Constants.kShooter.kDoubleClosedLoop.kFront.SETPOINT_OFFSET) * 600.0 /
    // 2048.0);
    // SmartDashboard.putNumber("back shooter setpoint",
    // (backSetpointRPMWithOffset -
    // Constants.kShooter.kDoubleClosedLoop.kBack.SETPOINT_OFFSET) * 600.0 /
    // 2048.0);
    SmartDashboard.putNumber("back shooter actual rpm", back.getSelectedSensorVelocity()
        * 600.0 / 2048.0);
    // frontChange = left.getSelectedSensorPosition();
    // back.set(ControlMode.PercentOutput, Constants.kShooter.kOpenLoop.BACK_SPEED);

    if (frontSetpointRPMWithOffset == 0) { // assume both setpoints are zero
      left.set(ControlMode.PercentOutput, 0);
      back.set(ControlMode.PercentOutput, 0);
    } else {
      left.set(ControlMode.Velocity, Constants.convertRPMtoTrans(frontSetpointRPMWithOffset));
      back.set(ControlMode.Velocity, Constants.convertRPMtoTrans(backSetpointRPMWithOffset));
    }
    // left.set(ControlMode.PercentOutput, fForward.calculate(setpoint) / 12);
  }

  public boolean isAtSpeed() {
    double frontDiff = Math.abs(Constants.convertRPMtoTrans(frontSetpointRPM.get()) - left.getSelectedSensorVelocity());
    double backDiff = Math.abs(Constants.convertRPMtoTrans(backSetpointRPM.get()) - back.getSelectedSensorVelocity());
    return frontDiff <= Constants.convertRPMtoTrans(Constants.kShooter.kDoubleClosedLoop.kFront.ERROR_TOLERANCE)
        && backDiff <= Constants.convertRPMtoTrans(Constants.kShooter.kDoubleClosedLoop.kBack.ERROR_TOLERANCE);
  }

  @Override
  public void simulationPeriodic() {
  }
}
