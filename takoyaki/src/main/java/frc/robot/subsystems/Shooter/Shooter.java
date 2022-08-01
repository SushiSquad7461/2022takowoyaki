// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableNumber;
import java.util.function.Supplier;

public abstract class Shooter extends SubsystemBase {

  public abstract void runShooter();

  public abstract void stopShooter();

  public abstract void runKicker();

  public abstract void stopKicker();

  public abstract void reverseKicker();

  public abstract void zeroSetpoint();

  public abstract boolean isAtSpeed();

  public abstract double getKickerOutput();

  public enum ShooterState {
    FENDER(1.0, 2.305),
    RANGED(1, 2.305),
    AUTO(0.8, 6.25),
    OUTREACH(1, 3),
    TUNABLE(new TunableNumber("Tunable shooter amp", 1.0), new TunableNumber("Tunable shooter ratio", 2.305));

    private Supplier<Double> amp;
    private Supplier<Double> ratio;

    private ShooterState(double amp, double ratio) {
      this.amp = () -> amp;
      this.ratio = () -> ratio;
    }

    private ShooterState(TunableNumber amp, TunableNumber ratio) {
      this.amp = () -> amp.get();
      this.ratio = () -> ratio.get();
    }

    public double getAmp() {
      return amp.get();
    }

    public double getRatio() {
      return ratio.get();
    }
  }

  protected ShooterState state;

  public void setState(ShooterState state) {
    this.state = state;
  };

  public ShooterState getState() {
    return state;
  };

  protected double getAmp() {
    return state.getAmp();
  }

  protected double getRatio() {
    return state.getRatio();
  }
}
