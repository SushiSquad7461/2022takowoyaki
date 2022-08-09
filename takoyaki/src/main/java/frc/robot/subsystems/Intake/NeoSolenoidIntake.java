// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class NeoSolenoidIntake extends Intake {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntake.MOTOR_ID, MotorType.kBrushless);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.kIntake.LEFT_SOLENOID_FORWARD, Constants.kIntake.LEFT_SOLENOID_REVERSE);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.kIntake.RIGHT_SOLENOID_FORWARD, Constants.kIntake.RIGHT_SOLENOID_REVERSE);

  public NeoSolenoidIntake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(Constants.kIntake.kSpark.CURRENT_LIMIT);

    // initial state
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void runIntakeMotor() {
    intakeMotor.set(Constants.kIntake.INTAKE_SPEED);
  }

  public void runIntakeMotorBackwards() {
    intakeMotor.set(-Constants.kIntake.INTAKE_SPEED);
  }

  public void intake() {
    rightSolenoid.set(Value.kForward);
    leftSolenoid.set(Value.kForward);
    runIntakeMotor();
  }

  public void stop() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    intakeMotor.set(0);
  }

  public void outtake() {
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
    runIntakeMotorBackwards();
  }

  // toggles intake extension
  public void toggleIntake() {
    if (rightSolenoid.get() == Value.kReverse) {
      intake();
    } else {
      stop();
    }
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
