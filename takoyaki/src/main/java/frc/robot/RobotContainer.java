// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController opController;
  private final Climb climb;

  public RobotContainer() {
    opController = new XboxController(1);
    climb = new Climb();

    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(opController, Constants.kClimb.CLIMB_TO_TOP_BUTTON)
        .whenPressed(new InstantCommand(climb::toggleExtendClimb, climb));

    new JoystickButton(opController, Constants.kClimb.CLIMB_TO_BOTTOM_BUTTON)
        .whenPressed(new InstantCommand(climb::toggleRetractClimb, climb));

    new JoystickButton(opController, Constants.kClimb.CLIMB_OPEN_LOOP_RAISE_BUTTON)
        .whenPressed(new RunCommand(climb::runOpenLoopClimb, climb))
        .whenReleased(new InstantCommand(climb::stopClimb, climb));

    new JoystickButton(opController, Constants.kClimb.CLIMB_OPEN_LOOP_LOWER_BUTTON)
        .whenPressed(new RunCommand(climb::reverseOpenLoopClimb, climb))
        .whenReleased(new InstantCommand(climb::stopClimb, climb));

    new JoystickButton(opController, Constants.kClimb.CLIMB_ENCODER_RESET_BUTTON)
        .whenPressed(new RunCommand(climb::zeroClimbEncoder, climb));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
