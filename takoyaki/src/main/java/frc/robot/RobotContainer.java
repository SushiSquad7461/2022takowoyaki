// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Hopper;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsytems, and commands
  private final Hopper hopper = new Hopper();

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
    // Run Intake
    new JoystickButton(driveController, Constants.kHopper.RUN_HOPPER)
      .whenPressed(new InstantCommand(hopper::runForward, hopper))
      .whenReleased(new InstantCommand(hopper::stop, hopper));

    // Reverse Intake
    new JoystickButton(driveController, Constants.kHopper.REVERSE_HOPPER)
      .whenPressed(new InstantCommand(hopper::runBackward, hopper))
      .whenReleased(new InstantCommand(hopper::stop, hopper));
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
