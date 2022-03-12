// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class Shooter extends SubsystemBase {
  protected ShooterState state;

  public abstract void runShooter();

  public abstract void stopShooter();

  public abstract void runKicker();

  public abstract void stopKicker();

  public abstract void reverseKicker();

  public abstract void zeroSetpoint();

  public abstract void setSetpoint();

  public abstract boolean isAtSpeed();

  public enum ShooterState {
    FENDER,
    EXIT
  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public ShooterState getState() {
    return this.state;
  }

  public ShooterState getStateFromVision(PhotonCamera camera) {
    double pitch = camera.getLatestResult().getBestTarget().getPitch();
    if (pitch < Constants.kShooter.kVision.EXIT_PITCH) {
      return ShooterState.EXIT;
    } else {
      return ShooterState.FENDER;
    }
  }

}
