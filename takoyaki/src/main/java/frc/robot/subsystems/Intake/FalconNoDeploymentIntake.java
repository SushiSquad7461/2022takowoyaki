// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FalconNoDeploymentIntake extends Intake {
  private final TalonFX intakeMotor = new TalonFX(Constants.kIntake.MOTOR_ID);

  public FalconNoDeploymentIntake() {
    intakeMotor.setInverted(Constants.kIntake.kFalcon.INVERT_TYPE);
  }

  public void runIntake() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, Constants.kIntake.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, 0);
  }

  public void reverseIntake() {
    runIntakeBackwards();
  }

  public void runIntakeBackwards() {
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, -Constants.kIntake.INTAKE_SPEED);
  }

  public void toggleIntake() {
    /*
     * if (solenoid.get() == Value.kReverse) {
     * solenoid.set(Value.kForward);
     * } else {
     * solenoid.set(Value.kReverse);
     * }
     */
    // cry
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
    intakeMotor.set(Constants.kIntake.kFalcon.CONTROL_MODE, Constants.kIntake.INTAKE_SPEED);
  }
}
