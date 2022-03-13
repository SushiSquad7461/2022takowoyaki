// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TunableNumber;

public abstract class Shooter extends SubsystemBase {
  public abstract void runShooter();

  public abstract void stopShooter();

  public abstract void runKicker();

  public abstract void stopKicker();

  public abstract void reverseKicker();

  public abstract void zeroSetpoint();

  public abstract void setSetpoint();

  public abstract boolean isAtSpeed();

  // shooter state stuff
  protected ShooterState state;

  public enum ShooterState {
    FENDER(
        new TunableNumber("fender front setpoint rpm", Constants.kShooter.FENDER_FRONT_SETPOINT_RPM),
        new TunableNumber("fender back setpoint rpm", Constants.kShooter.FENDER_BACK_SETPOINT_RPM)),

    FAR(new TunableNumber("far front setpoint rpm", Constants.kShooter.FAR_FRONT_SHOOTER_RPM),
        new TunableNumber("far back setpoint rpm", Constants.kShooter.FENDER_BACK_SHOOTER_RPM));

    public TunableNumber frontSetpointRPM;
    public TunableNumber backSetpointRPM;

    private ShooterState(TunableNumber frontSetpointRPM, TunableNumber backSetpointRPM) {
      this.frontSetpointRPM = frontSetpointRPM;
      this.backSetpointRPM = backSetpointRPM;
    }

  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public ShooterState getState() {
    return state;
  }

}
