// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.TalonHopper;
import frc.robot.subsystems.Hopper.VictorHopper;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.FalconDrivetrain;
import frc.robot.subsystems.Intake.FalconNoDeploymentIntake;
import frc.robot.subsystems.Intake.FalconSolenoidIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ClosedLoopFalconComp;
import frc.robot.subsystems.Shooter.ClosedLoopFalconShooter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Hopper hopper;
  private final Intake intake;
  private final Shooter shooter;
  private final Drivetrain drivetrain;

  // Controllers
  private final XboxController driveController;
  private final XboxController operatorController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    Constants.setup();
    hopper = new TalonHopper();
    intake = new FalconSolenoidIntake();
    shooter = new ClosedLoopFalconComp();
    drivetrain = new FalconDrivetrain();
    driveController = new XboxController(Constants.kOI.DRIVE_CONTROLLER);
    operatorController = new XboxController(Constants.kOI.OPERATOR_CONTROLLER);
    configureButtonBindings();
  }

  public void setDrivetrainToCoast() {
    drivetrain.setToCoastMode();
  }

  public void setDrivetrainToBrake() {
    drivetrain.setToBrakeMode();
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
    // run hopper
    new JoystickButton(driveController, Constants.kOI.RUN_HOPPER)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(shooter::runKicker, shooter),
            new RunCommand(hopper::runHopper, hopper)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, intake),
            new RunCommand(hopper::stop, hopper)));

    // reverse hopper
    new JoystickButton(driveController, Constants.kOI.REVERSE_HOPPER)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(shooter::reverseKicker, shooter),
            new RunCommand(hopper::reverseHopper, hopper)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, shooter),
            new RunCommand(hopper::stop, hopper)));

    // Actuate Intake
    /*
     * new JoystickButton(driveController, Constants.kOI.TOGGLE_INTAKE)
     * .whenPressed(new InstantCommand(intake::toggleIntake, intake));
     */

    // Run Intake
    new JoystickButton(driveController, Constants.kOI.RUN_INTAKE)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(intake::runIntake, intake)))
        // new RunCommand(hopper::runHopper, hopper)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(intake::stop, intake)));
    // new RunCommand(hopper::stop, hopper)));

    // Reverse Intake
    new JoystickButton(driveController, Constants.kOI.REVERSE_INTAKE)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(shooter::reverseKicker, shooter),
            new RunCommand(hopper::reverseHopper, hopper),
            new RunCommand(intake::reverseIntake, intake)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, shooter),
            new RunCommand(hopper::stop, hopper),
            new RunCommand(intake::stop, intake)));

    // Run Shooter
    new JoystickButton(driveController, Constants.kOI.RUN_SHOOTER)
        .whenPressed(new RunCommand(() -> shooter.setSetpoint(), shooter))
        .whenReleased(new RunCommand(() -> shooter.zeroSetpoint(), shooter));

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
