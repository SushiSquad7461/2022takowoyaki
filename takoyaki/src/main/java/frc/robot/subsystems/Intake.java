// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.kIntake.MOTOR_ID);
  //private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kIntake.SOLENOID_FRONT, Constants.kIntake.SOLENOID_BACK);

  public Intake() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(Constants.kIntake.INVERTED);
    
    //solenoid.set(DoubleSolenoid.Value.kOff);
    //solenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void runIntake() {
    actuateIntake();
    intakeMotor.set(Constants.kIntake.INTAKE_SPEED);
  }

  public void stop() {
    retractIntake();
    intakeMotor.set(0);
  }

  public void reverseIntake() {
    actuateIntake();
    intakeMotor.set(-Constants.kIntake.INTAKE_SPEED);
  }

  // public void toggleIntake() {
  //   if(solenoid.get() == Value.kReverse) {
  //     solenoid.set(Value.kForward);
  //   } else {
  //     solenoid.set(Value.kReverse);
  //   }
  // }

  public void actuateIntake() {
    //solenoid.set(Value.kReverse);
  }

  public void retractIntake() {
    //solenoid.set(Value.kForward);
  }

  @Override
  public void periodic() { }

  @Override
  public void simulationPeriodic() { }
}
