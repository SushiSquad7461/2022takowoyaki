// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.kIntake.MOTOR_ID);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, Constants.kIntake.LEFT_SOLENOID_FRONT, Constants.kIntake.LEFT_SOLENOID_BACK);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, Constants.kIntake.RIGHT_SOLENOID_FRONT, Constants.kIntake.RIGHT_SOLENOID_BACK);

  public Intake() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(TalonFXInvertType.CounterClockwise);
    
    leftSolenoid.set(DoubleSolenoid.Value.kOff);
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kOff);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void runIntake() {
    toggleIntake();
    intakeMotor.set(Constants.kIntake.INTAKE_SPEED);
  }

  public void stop() {
    toggleIntake();
    intakeMotor.set(0);
  }

  public void reverseIntake() {
    toggleIntake();
    intakeMotor.set(-Constants.kIntake.INTAKE_SPEED);
  }

  public void toggleIntake() {
    if(leftSolenoid.get() == Value.kReverse) {
      leftSolenoid.set(Value.kForward);
      rightSolenoid.set(Value.kForward);
    } else {
      leftSolenoid.set(Value.kReverse);
      rightSolenoid.set(Value.kReverse);
    }
  }

  public void actuateIntake() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  public void retractIntake() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() { }

  @Override
  public void simulationPeriodic() { }
}
