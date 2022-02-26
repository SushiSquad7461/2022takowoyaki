// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Shooter.Shooter;

/** An example command that uses an example subsystem. */
public class AutoShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter s_shooter;
  private final Hopper s_hopper;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot(Shooter shooter, Hopper hopper) {
    s_shooter = shooter;
    s_hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_shooter.setSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_shooter.isAtSpeed()) {
      s_hopper.runHopper();
      s_shooter.runKicker();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_hopper.stop();
    s_shooter.stopKicker();
    s_shooter.setSetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
