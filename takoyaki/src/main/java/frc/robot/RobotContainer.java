// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.TalonHopper;
import frc.robot.subsystems.Hopper.VictorHopper;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.FalconDrivetrain;
import frc.robot.subsystems.Intake.FalconNoDeploymentIntake;
import frc.robot.subsystems.Intake.FalconSolenoidIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ClosedLoopFalconComp;
import frc.robot.subsystems.Shooter.ClosedLoopFalconShooter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  // auto stuff
  private final Ramsete ramsete;
  private final Field2d field;
  private final AutoCommandSelector autoSelector;
  private SendableChooser<SequentialCommandGroup> autoChooser;

  public RobotContainer() {
    Constants.setup();
    hopper = new TalonHopper();
    intake = new FalconSolenoidIntake();
    shooter = new ClosedLoopFalconComp();
    drivetrain = new FalconDrivetrain();
    driveController = new XboxController(Constants.kOI.DRIVE_CONTROLLER);
    operatorController = new XboxController(Constants.kOI.OPERATOR_CONTROLLER);

    ramsete = new Ramsete(drivetrain);
    autoChooser = new SendableChooser<>();
    autoSelector = new AutoCommandSelector(drivetrain, ramsete, intake, shooter, hopper);
    field = new Field2d();

    autoChooser.setDefaultOption("two ball mid", autoSelector.twoBallMid);
    autoChooser.addOption("two ball far", autoSelector.twoBallFar);
    autoChooser.addOption("two ball wall", autoSelector.twoBallWall);
    autoChooser.addOption("three ball", autoSelector.threeBall);
    autoChooser.addOption("five ball", autoSelector.fiveBall);
    autoChooser.addOption("reverse testig", autoSelector.reverseSpline);
    autoChooser.addOption("iota five ball", autoSelector.iotaFiveBall);
    autoChooser.addOption("zeta five ball", autoSelector.zetaFiveBall);
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
            new RunCommand(shooter::runKicker, shooter),
            new RunCommand(hopper::runHopper, hopper)))
        .whenReleased(new ParallelCommandGroup(
            new RunCommand(shooter::stopKicker, intake),
            new RunCommand(hopper::stop, hopper)));

    // reverse s_hopper
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

    //drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.curveDrive(OI.getTriggers(driveController),
    //    OI.getLeftStick(driveController), driveController.getXButton()), drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateRobotPose() {
    field.setRobotPose(drivetrain.getPose());
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
    drivetrain.setBrake();
  }

  public void setDriveCoast() {
    drivetrain.setCoast();
  }

  public void tankDriveVolts(int left, int right) {
    drivetrain.tankDriveVolts(left, right);
  }

}
