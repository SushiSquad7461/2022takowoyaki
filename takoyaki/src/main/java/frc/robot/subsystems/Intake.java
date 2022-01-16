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
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX falcon = new WPI_TalonFX(Constants.kIntake.FALCON_MOTOR_ID);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kIntake.SOLENOID_FRONT, Constants.kIntake.SOLENOID_BACK);

  public Intake() {
    falcon.configFactoryDefault();

    falcon.setInverted(TalonFXInvertType.Clockwise);
    
    solenoid.set(DoubleSolenoid.Value.kOff);
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  
  public void startIntake() {
    falcon.set(ControlMode.PercentOutput, Constants.kIntake.INTAKE_SPEED);
  }

  public void stopIntake() {
    falcon.set(ControlMode.PercentOutput, 0);
  }

  public void startReverse() {
    falcon.set(ControlMode.PercentOutput, -Constants.kIntake.INTAKE_SPEED);
  }

  public void actuateIntake() {
    solenoid.set(Value.kReverse);
  }

  public void retractIntake() {
    solenoid.set(Value.kForward);
  }

  @Override
  public void periodic() { }

  @Override
  public void simulationPeriodic() { }
}
