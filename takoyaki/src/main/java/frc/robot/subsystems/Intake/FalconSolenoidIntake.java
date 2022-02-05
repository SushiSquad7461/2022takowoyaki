// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

public class FalconSolenoidIntake extends Intake {
  private final TalonFX intakeMotor = new TalonFX(Constants.kIntake.MOTOR_ID);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.kIntake.SOLENOID_FRONT, Constants.kIntake.SOLENOID_BACK);

  public FalconSolenoidIntake() {
    intakeMotor.setInverted(Constants.kIntake.kFalcon.INVERT_TYPE);

    solenoid.set(DoubleSolenoid.Value.kOff);
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void runIntake() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, Constants.kIntake.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, 0);
  }

  public void reverseIntake() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, -Constants.kIntake.INTAKE_SPEED);
  }

  public void toggleIntake() {
    if (solenoid.get() == Value.kReverse) {
      solenoid.set(Value.kForward);
    } else {
      solenoid.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
