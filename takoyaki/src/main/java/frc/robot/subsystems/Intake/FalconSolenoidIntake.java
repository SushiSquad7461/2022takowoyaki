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

public class FalconSolenoidIntake extends Intake {
  private final TalonFX intakeMotor = new TalonFX(Constants.kIntake.MOTOR_ID);
  private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.kIntake.LEFT_SOLENOID_FORWARD, Constants.kIntake.LEFT_SOLENOID_REVERSE);
  private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.kIntake.RIGHT_SOLENOID_FORWARD, Constants.kIntake.RIGHT_SOLENOID_REVERSE);

  public FalconSolenoidIntake() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.setInverted(Constants.kIntake.kFalcon.INVERT_TYPE);
    intakeMotor.configSupplyCurrentLimit(Constants.currentLimit((Constants.kIntake.kFalcon.CURRENT_LIMIT)));

    // initial state
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void runIntakeMotor() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, Constants.kIntake.INTAKE_SPEED);
  }

  public void runIntakeMotorBackwards() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, -Constants.kIntake.INTAKE_SPEED);
  }

  public void intake() {
    rightSolenoid.set(Value.kForward);
    leftSolenoid.set(Value.kForward);
    runIntakeMotor();
  }

  public void stop() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, 0);
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
