// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

/** An example command that uses an example subsystem. */
public class TunableAutoShoot extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;

  private Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TunableAutoShoot(Shooter shooter, Hopper hopper, Intake intake) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    timer = new Timer();
    addRequirements(shooter, hopper, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSetpoint();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isAtSpeed()) {
      hopper.runHopper();
      if ((timer.get() % 0.5) > 0.25) {
        shooter.runKicker();
      } else {
        shooter.stopKicker();
      }
      intake.runIntakeMotor();
    } else {
      hopper.stop();
      intake.stop();
      shooter.stopKicker();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
    shooter.stopKicker();
    shooter.zeroSetpoint();
    intake.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}