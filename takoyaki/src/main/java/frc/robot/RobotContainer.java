// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Hopper s_hopper;
  private final Intake s_intake;
  private final Drivetrain s_drivetrain;
  private final Shooter s_shooter;
  
  // Controllers
  private final XboxController driveController = new XboxController(Constants.kOI.DRIVE_CONTROLLER);
  private final XboxController operatorController = new XboxController(Constants.kOI.OPERATOR_CONTROLLER);

  // auto stuff
  private final Ramsete ramsete;
  private final Field2d field;
  private final AutoCommandSelector autoSelector;
  private SendableChooser<SequentialCommandGroup> autoChooser;

  public RobotContainer() {
    s_hopper = new Hopper();
    s_intake = new Intake();
    s_drivetrain = new Drivetrain();
    s_shooter = new Shooter();

    ramsete = new Ramsete(s_drivetrain);
    autoChooser = new SendableChooser<>();
    autoSelector = new AutoCommandSelector(s_drivetrain, ramsete, s_intake, s_shooter, s_hopper);
    field = new Field2d();

    autoChooser.setDefaultOption("two ball mid", autoSelector.twoBallMid);
    autoChooser.addOption("two ball far", autoSelector.twoBallFar);
    autoChooser.addOption("two ball wall", autoSelector.twoBallWall);
    autoChooser.addOption("three ball", autoSelector.threeBall);
    autoChooser.addOption("five ball", autoSelector.fiveBall);
    autoChooser.addOption("reverse testig", autoSelector.reverseSpline);
    // put field object to dashboard
    SmartDashboard.putData("field", field);

    // set up chooser
    SmartDashboard.putData("auto options", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // run s_hopper
    new JoystickButton(driveController, Constants.kOI.RUN_HOPPER)
      .whenPressed(new ParallelCommandGroup(
        new InstantCommand(s_shooter::runKicker, s_intake),
        new InstantCommand(s_hopper::runHopper, s_hopper)))
      .whenReleased(new ParallelCommandGroup(
        new InstantCommand(s_shooter::stopKicker, s_intake),
        new InstantCommand(s_hopper::stop, s_hopper)));

    // reverse s_hopper
    new JoystickButton(driveController, Constants.kOI.REVERSE_HOPPER)
      .whenPressed(new ParallelCommandGroup(
        new InstantCommand(s_intake::actuateIntake, s_intake)
        .andThen(new InstantCommand(s_intake::reverseIntake, s_intake)),
        new InstantCommand(s_hopper::reverseHopper, s_hopper)))
      .whenReleased(new ParallelCommandGroup(
        new InstantCommand(s_intake::retractIntake, s_intake)
        .andThen(new InstantCommand(s_intake::stop, s_intake)),
        new InstantCommand(s_hopper::stop, s_hopper)));

    // Actuate Intake
    // new JoystickButton(driveController, Constants.kOI.TOGGLE_INTAKE)
    //   .whenPressed(new InstantCommand(s_intake::toggleIntake, s_intake));
      
    // Run Intake
    new JoystickButton(driveController, Constants.kOI.RUN_INTAKE)
      .whenPressed(new ParallelCommandGroup(
        new InstantCommand(s_intake::actuateIntake, s_intake)
        .andThen(new InstantCommand(s_intake::runIntake, s_intake)),
        new InstantCommand(s_hopper::runHopper, s_hopper)))
      .whenReleased(new ParallelCommandGroup(
        new InstantCommand(s_intake::retractIntake, s_intake)
        .andThen(new InstantCommand(s_intake::stop, s_intake)),
        new InstantCommand(s_hopper::stop, s_hopper)));

    // Reverse Intake
    new JoystickButton(driveController, Constants.kOI.REVERSE_INTAKE)
      .whenPressed(new InstantCommand(s_intake::reverseIntake, s_intake))
      .whenReleased(new InstantCommand(s_intake::stop, s_intake));

    // Run Shooter
    new JoystickButton(driveController, Constants.kOI.RUN_SHOOTER)
      .whenPressed(new InstantCommand(s_shooter::runShooter, s_shooter))
      .whenReleased(new InstantCommand(s_shooter::stopShooter));
      
    //s_drivetrain.setDefaultCommand(new RunCommand(() -> s_drivetrain.curveDrive(OI.getTriggers(driveController),
    //  OI.getLeftStick(driveController), driveController.getXButton()), s_drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateRobotPose() {
    field.setRobotPose(s_drivetrain.getPose());
  }

  public void setFieldTrajectory() {
    Trajectory concatTrajectory = new Trajectory();
    for(RamsetePath p : autoSelector.pathArrayMap.get(autoChooser.getSelected())) {
      concatTrajectory = concatTrajectory.concatenate(p.getTrajectory());
    }
    field.getObject(Constants.kOI.TRAJECTORY_NAME).setTrajectory(concatTrajectory);
  }

  public void setInitialPose() {
    autoSelector.setInitialDrivePose(autoChooser.getSelected());
  }

  public void setDriveBrake() {
    s_drivetrain.setBrake();
  }

  public void setDriveCoast() {
    s_drivetrain.setCoast();
  }

}
