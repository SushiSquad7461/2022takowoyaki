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
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Hopper s_hopper;
  private final Intake s_intake;
  private final Drivetrain s_drivetrain;
  
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

    ramsete = new Ramsete(s_drivetrain);
    autoChooser = new SendableChooser<>();
    autoSelector = new AutoCommandSelector(s_drivetrain, ramsete, s_intake);
    field = new Field2d();

    autoChooser.setDefaultOption("two ball mid", autoSelector.twoBallMid);
    autoChooser.addOption("two ball far", autoSelector.twoBallFar);
    autoChooser.addOption("two ball wall", autoSelector.twoBallWall);
    autoChooser.addOption("three ball", autoSelector.threeBall);
    autoChooser.addOption("five ball", autoSelector.fiveBall);

    // put field object to dashboard
    SmartDashboard.putData("field", field);

    // set up chooser
    SmartDashboard.putData("auto options", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // run hopper
    new JoystickButton(driveController, Constants.kHopper.RUN_HOPPER)
      .whenPressed(new InstantCommand(s_hopper::runForward, s_hopper))
      .whenReleased(new InstantCommand(s_hopper::stop, s_hopper));

    // reverse hopper
    new JoystickButton(driveController, Constants.kHopper.REVERSE_HOPPER)
      .whenPressed(new InstantCommand(s_hopper::runBackward, s_hopper))
      .whenReleased(new InstantCommand(s_hopper::stop, s_hopper));

    // Actuate Intake
    new JoystickButton(driveController, Constants.kIntake.ACTUATE_INTAKE)
      .whenPressed(new InstantCommand(s_intake::actuateIntake, s_intake));

    // Retract Intake
    new JoystickButton(driveController, Constants.kIntake.RETRACT_INTAKE)
      .whenPressed(new InstantCommand(s_intake::retractIntake, s_intake));
      
    // Run Intake
    new JoystickButton(driveController, Constants.kIntake.RUN_INTAKE)
      .whenPressed(new InstantCommand(s_intake::startIntake, s_intake))
      .whenReleased(new InstantCommand(s_intake::stopIntake, s_intake));

    // Reverse Intake
    new JoystickButton(driveController, Constants.kIntake.REVERSE_INTAKE)
      .whenPressed(new InstantCommand(s_intake::startReverse, s_intake))
      .whenReleased(new InstantCommand(s_intake::stopIntake, s_intake));

    s_drivetrain.setDefaultCommand(new RunCommand(() -> s_drivetrain.curveDrive(OI.getTriggers(driveController),
      OI.getLeftStick(driveController), driveController.getXButton()), s_drivetrain));
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

}
