// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Hopper hopper = new Hopper();
  private final Intake intake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();
  
  // Controllers
  private final XboxController driveController = new XboxController(Constants.kOI.DRIVE_CONTROLLER);
  private final XboxController operatorController = new XboxController(Constants.kOI.OPERATOR_CONTROLLER);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // run hopper
    new JoystickButton(driveController, Constants.kHopper.RUN_HOPPER)
      .whenPressed(new InstantCommand(hopper::runForward, hopper))
      .whenReleased(new InstantCommand(hopper::stop, hopper));

    // reverse hopper
    new JoystickButton(driveController, Constants.kHopper.REVERSE_HOPPER)
      .whenPressed(new InstantCommand(hopper::runBackward, hopper))
      .whenReleased(new InstantCommand(hopper::stop, hopper));

    // Actuate Intake
    new JoystickButton(driveController, Constants.kIntake.ACTUATE_INTAKE)
      .whenPressed(new InstantCommand(intake::actuateIntake, intake));

    // Retract Intake
    new JoystickButton(driveController, Constants.kIntake.RETRACT_INTAKE)
      .whenPressed(new InstantCommand(intake::retractIntake, intake));
      
    // Run Intake
    new JoystickButton(driveController, Constants.kIntake.RUN_INTAKE)
      .whenPressed(new InstantCommand(intake::startIntake, intake))
      .whenReleased(new InstantCommand(intake::stopIntake, intake));

    // Reverse Intake
    new JoystickButton(driveController, Constants.kIntake.REVERSE_INTAKE)
      .whenPressed(new InstantCommand(intake::startReverse, intake))
      .whenReleased(new InstantCommand(intake::stopIntake, intake));

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.curveDrive(OI.getTriggers(driveController),
      OI.getLeftStick(driveController), driveController.getXButton()), drivetrain));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
