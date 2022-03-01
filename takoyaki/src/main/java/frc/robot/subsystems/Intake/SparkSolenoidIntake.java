// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class SparkSolenoidIntake extends Intake {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntake.MOTOR_ID,
      Constants.kIntake.kSpark.MOTOR_TYPE);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.kIntake.LEFT_SOLENOID_FORWARD, Constants.kIntake.LEFT_SOLENOID_REVERSE);

  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.kIntake.RIGHT_SOLENOID_FORWARD, Constants.kIntake.RIGHT_SOLENOID_REVERSE);

  public SparkSolenoidIntake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.kIntake.kSpark.INVERTED);
    intakeMotor.setOpenLoopRampRate(Constants.kIntake.kSpark.OPEN_LOOP_RAMP_RATE);
    intakeMotor.setSmartCurrentLimit(Constants.kIntake.kSpark.CURRENT_LIMIT);
    intakeMotor.burnFlash();

    leftSolenoid.set(DoubleSolenoid.Value.kOff);
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);

    rightSolenoid.set(DoubleSolenoid.Value.kOff);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void runIntake() {
    intakeMotor.set(Constants.kIntake.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void reverseIntake() {
    intakeMotor.set(-Constants.kIntake.INTAKE_SPEED);
  }

  public void toggleIntake() {
    // if (solenoid.get() == Value.kReverse) {
    //   solenoid.set(Value.kForward);
    // } else {
    //   solenoid.set(Value.kReverse);
    // }
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public void intakeShoot() {
    // leftSolenoid.set(DoubleSolenoid.Value.kForward);
    // rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }
}
