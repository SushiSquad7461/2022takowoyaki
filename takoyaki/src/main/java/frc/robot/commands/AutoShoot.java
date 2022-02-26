// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class AutoShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter s_shooter;
  private final Hopper s_hopper;
  private final Intake s_intake;

  private int shots;
  private boolean on_ball;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot(Shooter shooter, Hopper hopper, Intake intake) {
    s_shooter = shooter;
    s_hopper = hopper;
    s_intake = intake;
    shots = 0;
    on_ball = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hopper, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_shooter.setGoal();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_shooter.isAtSpeed()) {
      s_hopper.runHopper();
      s_shooter.runKicker();
      s_intake.runIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_hopper.stop();
    s_shooter.stopKicker();
    s_shooter.setGoal(0);
    s_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shots>=2 && !on_ball) {
      return true;
    } else if (!on_ball && s_shooter.beamBroken()) {
      shots++;
      on_ball = true;
    } else if (on_ball && !s_shooter.beamBroken()) {
      on_ball = false;
    }
    return false;
  }
}
