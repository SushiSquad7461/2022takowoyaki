// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kTurnToTarget;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

/** An example command that uses an example subsystem. */
public class AutoShootWithVision extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private ShooterState state = ShooterState.RANGED;

  private Timer timer;
  private PhotonCamera camera;
  private Drivetrain drivetrain;
  private PIDController PID;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShootWithVision(Shooter shooter, Hopper hopper, Intake intake, Drivetrain drivetrain,
      PhotonCamera camera) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.camera = camera;
    this.drivetrain = drivetrain;
    this.PID = new PIDController(kTurnToTarget.kP, kTurnToTarget.kI, kTurnToTarget.kD);

    timer = new Timer();
    addRequirements(shooter, hopper, intake);
  }

  public AutoShootWithVision(Shooter shooter, Hopper hopper, Intake intake, ShooterState state, Drivetrain drivetrain,
      PhotonCamera camera) {
    this(shooter, hopper, intake, drivetrain, camera);
    this.state = state;
  }

  public double getYawFromAverage(List<PhotonTrackedTarget> targets) {
    double result = 0;
    for (PhotonTrackedTarget target : targets) {
      result += target.getYaw();
    }
    return result / targets.size();
  }

  public double getYawFromHighest(List<PhotonTrackedTarget> targets) {
    double highestPitch = 0;
    double highestYaw = Double.MIN_VALUE;
    for (PhotonTrackedTarget target : targets) {
      if (highestYaw < target.getYaw()) {
        highestPitch = target.getPitch();
      }
    }
    return highestPitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setState(state);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (shooter.isAtSpeed()) {
    // hopper.runHopper();
    // if ((timer.get() % 0.5) > 0.25) {
    // shooter.runKicker();
    // } else {
    // shooter.stopKicker();
    // }
    // intake.runIntakeMotor();
    // } else {
    // hopper.stop();
    // intake.stop();
    // shooter.stopKicker();
    // }
    PhotonPipelineResult result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    double yaw = getYawFromAverage(targets); // change this to getYawFromHighest maybe ?!?!
    double output = PID.calculate(yaw, 0);
    drivetrain.curveDrive(0, output, true);

    if (shooter.isAtSpeed() && Math.abs(output) < Constants.kTurnToTarget.ERROR_TOLLERANCE) {
      hopper.runHopper();
      intake.runIntakeMotor();
      shooter.runKicker();
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
    drivetrain.curveDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
