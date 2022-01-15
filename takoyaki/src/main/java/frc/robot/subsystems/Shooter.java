// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX left = new WPI_TalonFX(Constants.kShooter.LEFT_MOTOR_ID);
  private WPI_TalonFX right = new WPI_TalonFX(Constants.kShooter.RIGHT_MOTOR_ID);

  public Shooter() {
    left.configFactoryDefault();
    right.configFactoryDefault();

    left.setNeutralMode(NeutralMode.Brake);
		right.setNeutralMode(NeutralMode.Brake);

    left.setInverted(TalonFXInvertType.Clockwise);
    right.setInverted(TalonFXInvertType.CounterClockwise);

  }

  public void run() {
    left.set(ControlMode.PercentOutput, Constants.kShooter.SPEED);
    right.set(ControlMode.PercentOutput, Constants.kShooter.SPEED);
  }


  @Override
  public void periodic() { }

  @Override
  public void simulationPeriodic() { }
}
