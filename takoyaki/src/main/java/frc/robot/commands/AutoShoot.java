// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

/** An example command that uses an example subsystem. */
public class AutoShoot extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Supplier<ShooterState> stateSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot(Shooter shooter, Hopper hopper, Intake intake, Supplier<ShooterState> stateSupplier) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.stateSupplier = stateSupplier;
    addRequirements(shooter, hopper, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setState(stateSupplier.get());
    shooter.setSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setState(stateSupplier.get());
    SmartDashboard.putString("Shooter state", stateSupplier.get().toString());
    shooter.setSetpoint();
    if (shooter.isAtSpeed()) {
      hopper.runHopper();
      shooter.runKicker();
      intake.runIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
    shooter.stopKicker();
    shooter.zeroSetpoint();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
