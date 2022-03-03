// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.FalconBrakeModeClimb;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.TalonHopper;
import frc.robot.subsystems.Hopper.VictorHopper;
import frc.robot.commands.AutoShoot;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.FalconDrivetrain;
import frc.robot.subsystems.Intake.FalconNoDeploymentIntake;
import frc.robot.subsystems.Intake.FalconSolenoidIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ClosedLoopFalconComp;
import frc.robot.subsystems.Shooter.ClosedLoopFalconShooter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.XboxController;
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
    private final Climb climb;

    // Controllers
    private final XboxController driveController;
    private final XboxController operatorController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Constants.setup();

        // subsystems
        hopper = new TalonHopper();
        intake = new FalconSolenoidIntake();
        shooter = new ClosedLoopFalconComp();
        drivetrain = new FalconDrivetrain();
        climb = new FalconBrakeModeClimb();
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
    /*
     * new JoystickButton(operatorController, Constants.kClimb.CLIMB_TO_TOP_BUTTON)
     * .whenPressed(new InstantCommand(climb::extendClimb, climb));
     * 
     * new JoystickButton(operatorController,
     * Constants.kClimb.CLIMB_TO_BOTTOM_BUTTON)
     * .whenPressed(new InstantCommand(climb::retractClimb, climb));
     */

    new JoystickButton(operatorController, Constants.kClimb.CLIMB_TO_TOP_BUTTON)
        .whenPressed(new RunCommand(climb::runClimb, climb))
        .whenReleased(new InstantCommand(climb::stopClimb, climb));

    new JoystickButton(operatorController, Constants.kClimb.CLIMB_TO_BOTTOM_BUTTON)
        .whenPressed(new RunCommand(climb::climbDown, climb))
        .whenReleased(new InstantCommand(climb::stopClimb, climb));

    new JoystickButton(operatorController, Constants.kClimb.SEPARATE_CLIMB)
        .whenPressed(new InstantCommand(climb::separateClimb, climb));

    new JoystickButton(operatorController, Constants.kClimb.REJOIN_CLIMB)
        .whenPressed(new InstantCommand(climb::rejoinClimb, climb));

    // .whenPressed(new InstantCommand(climb::runOpenLoopClimb, climb))
    // .whenReleased(new InstantCommand(climb::stopClimb, climb));

    // new JoystickButton(operatorController,
    // Constants.kClimb.CLIMB_OPEN_LOOP_LOWER_BUTTON)
    // .whenPressed(new InstantCommand(climb::reverseOpenLoopClimb, climb))
    // .whenReleased(new InstantCommand(climb::stopClimb, climb));

    new JoystickButton(operatorController, Constants.kClimb.CLIMB_ENCODER_RESET_BUTTON)
        .whenPressed(new RunCommand(climb::zeroClimbEncoders, climb));

    climb.setDefaultCommand(
        new RunCommand(() -> climb.defaultCommand(operatorController.getLeftY(),
            operatorController.getRightY()),
            climb));
    // run hopper
    new JoystickButton(driveController, Constants.kOI.RUN_HOPPER)
    new JoystickButton(driveController, Constants.kOI.SHOOT)
        .whenHeld(new AutoShoot(shooter, hopper, intake));

    // shoot ball (hopper + kicker)
    /*new JoystickButton(driveController, Constants.kOI.SHOOT)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(shooter::runKicker, shooter),
            new RunCommand(hopper::runHopper, hopper)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, intake),
            new RunCommand(hopper::stop, hopper)));*/

    // invert drive direction
    new JoystickButton(driveController, Constants.kOI.INVERT_DRIVE)
        .whenPressed(new InstantCommand(drivetrain::invertDrive, drivetrain));

    // reverse shoot (hopper + kicker)
    new JoystickButton(driveController, Constants.kOI.REVERSE_SHOOT)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(shooter::reverseKicker, shooter),
            new RunCommand(hopper::reverseHopper, hopper)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, shooter),
            new RunCommand(hopper::stop, hopper)));

    // toggle intake
    new JoystickButton(driveController, Constants.kOI.TOGGLE_INTAKE)
        .whenPressed(new InstantCommand(intake::toggleIntake, intake));

    // reverse intake
    new JoystickButton(driveController, Constants.kOI.REVERSE_INTAKE)
        .whenPressed(new ParallelCommandGroup(
            new RunCommand(shooter::reverseKicker, shooter),
            new RunCommand(hopper::reverseHopper, hopper),
            new RunCommand(intake::reverseIntake, intake)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, shooter),
            new RunCommand(hopper::stop, hopper),
            new RunCommand(intake::stop, intake)));

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.curveDrive(OI.getTriggers(driveController),
        OI.getLeftStick(driveController), driveController.getXButton()), drivetrain));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

}
