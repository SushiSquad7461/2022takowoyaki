// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntake.MOTOR_ID, Constants.kIntake.MOTOR_TYPE);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kIntake.SOLENOID_FRONT, Constants.kIntake.SOLENOID_BACK);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.kIntake.INVERTED);
    intakeMotor.setOpenLoopRampRate(Constants.kIntake.OPEN_LOOP_RAMP_RATE);
    intakeMotor.setSmartCurrentLimit(Constants.kIntake.CURRENT_LIMIT);
    intakeMotor.burnFlash();
    
    solenoid.set(DoubleSolenoid.Value.kOff);
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void startIntake() {
    intakeMotor.set(Constants.kIntake.INTAKE_SPEED);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void startReverse() {
    intakeMotor.set(-Constants.kIntake.INTAKE_SPEED);
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
