// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Climb extends SubsystemBase {

  private final WPI_TalonFX left;
  private final WPI_TalonFX right;
  private final DoubleSolenoid brakeSolenoid;

  public Climb() {

    left = new WPI_TalonFX(Constants.kClimb.LEFT_MOTOR_CAN_ID);
    right = new WPI_TalonFX(Constants.kClimb.RIGHT_MOTOR_CAN_ID);
    brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kClimb.SOLENOID_FRONT,
        Constants.kClimb.SOLENOID_BACK);

    brakeSolenoid.set(DoubleSolenoid.Value.kOff);

    left.configFactoryDefault();
    right.configFactoryDefault();

    left.follow(right);

    right.setInverted(TalonFXInvertType.Clockwise);
    left.setInverted(TalonFXInvertType.OpposeMaster);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
  }

  public void runClimb() {
    right.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_UP_MAX_POWER);
  }

  public void reverseClimb() {
    right.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_DOWN_MAX_POWER);
  }

  public void stopClimb() {
    right.set(ControlMode.PercentOutput, 0);
  }

  public void climbBrakeOn() {
    brakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void climbBrakeOff() {
    brakeSolenoid.set(DoubleSolenoid.Value.kOff);
  }
}
