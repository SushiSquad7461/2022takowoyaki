// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Hopper hopper = new Hopper();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Drivetrain drivetrain = new Drivetrain();
  
  // Controllers
  private final XboxController driveController = new XboxController(Constants.kOI.DRIVE_CONTROLLER);
  private final XboxController operatorController = new XboxController(Constants.kOI.OPERATOR_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // run hopper
    new JoystickButton(driveController, Constants.kOI.RUN_HOPPER)
      .whenPressed(new ParallelCommandGroup(
        new InstantCommand(shooter::runKicker, intake),
        new InstantCommand(hopper::runHopper, hopper)))
      .whenReleased(new ParallelCommandGroup(
        new InstantCommand(shooter::stopKicker, intake),
        new InstantCommand(hopper::stop, hopper)));

    // reverse hopper
    new JoystickButton(driveController, Constants.kOI.REVERSE_HOPPER)
      .whenPressed(new ParallelCommandGroup(
        new InstantCommand(intake::actuateIntake, intake),
        new InstantCommand(intake::reverseIntake, intake),
        new InstantCommand(hopper::reverseHopper, hopper)))
      .whenReleased(new ParallelCommandGroup(
        new InstantCommand(intake::retractIntake, intake),
        new InstantCommand(intake::stop, intake),
        new InstantCommand(hopper::stop, hopper)));

    // Actuate Intake
    // new JoystickButton(driveController, Constants.kOI.TOGGLE_INTAKE)
    //   .whenPressed(new InstantCommand(intake::toggleIntake, intake));
      
    // Run Intake
    new JoystickButton(driveController, Constants.kOI.RUN_INTAKE)
      .whenPressed(new ParallelCommandGroup(
        new InstantCommand(intake::actuateIntake, intake),
        new InstantCommand(intake::runIntake, intake),
        new InstantCommand(hopper::runHopper, hopper)))
      .whenReleased(new ParallelCommandGroup(
        new InstantCommand(intake::retractIntake, intake),
        new InstantCommand(intake::stop, intake),
        new InstantCommand(hopper::stop, hopper)));

    // Reverse Intake
    new JoystickButton(driveController, Constants.kOI.REVERSE_INTAKE)
      .whenPressed(new InstantCommand(intake::reverseIntake, intake))
      .whenReleased(new InstantCommand(intake::stop, intake));

    // Run Shooter
    new JoystickButton(driveController, Constants.kOI.RUN_SHOOTER)
      .whenPressed(new InstantCommand(shooter::runShooter, shooter))
      .whenReleased(new InstantCommand(shooter::stopShooter));

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.curveDrive(OI.getTriggers(driveController),
      OI.getLeftStick(driveController), driveController.getXButton()), drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
