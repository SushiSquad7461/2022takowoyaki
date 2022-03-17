// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.ldap.ExtendedRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.FalconBrakeModeClimb;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.TalonHopper;
import frc.robot.subsystems.Hopper.VictorHopper;
import frc.robot.Ramsete.RamsetePath;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ExtendClimb;
import frc.robot.commands.RetractClimb;
import frc.robot.commands.RangedAutoShoot;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.FalconDrivetrain;
import frc.robot.subsystems.Intake.FalconSolenoidIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ClosedLoopDoubleFalconShooter;
import frc.robot.subsystems.Shooter.OpenLoopDoubleFalconShooter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

        // The robot's subsystems and commands are defined here...
        public final Hopper hopper;
        public final Intake intake;
        public final Shooter shooter;
        public final Drivetrain drivetrain;
        public final Climb climb;
        // private final Climb climb;

        // controllers
        private final XboxController driveController;
        private final XboxController operatorController;

        // auto stuff
        private final Ramsete ramsete;
        private final Field2d field;
        private final AutoCommandSelector autoSelector;
        private SendableChooser<SequentialCommandGroup> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                Constants.setup();

                hopper = new TalonHopper();
                intake = new FalconSolenoidIntake();
                shooter = new ClosedLoopDoubleFalconShooter();
                drivetrain = new FalconDrivetrain();
                climb = new FalconBrakeModeClimb();
                // climb = new FalconBrakeModeClimb();

                // controllers
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

                new JoystickButton(driveController, Constants.kOI.SHOOT)
                                .whenHeld(new AutoShoot(shooter, hopper, intake));

                CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);

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
                new JoystickButton(operatorController, Constants.kOI.TRAVERSAL_CLIMB)
                                .whenPressed(new ExtendClimb(climb));
                new JoystickButton(operatorController, Constants.kOI.MID_CLIMB)
                                .whenPressed(new RetractClimb(climb));
                new JoystickButton(operatorController, Constants.kOI.OPEN_LOOP_RAISE_CLIMB)
                                .whenPressed(climb::runClimb, climb).whenReleased(climb::stopClimb, climb);
                new JoystickButton(operatorController, Constants.kOI.OPEN_LOOP_LOWER_CLIMB)
                                .whenPressed(climb::reverseClimb, climb).whenReleased(climb::stopClimb, climb);
               
                new JoystickButton(driveController, Constants.kOI.SHOOT)
                                .whenHeld(new AutoShoot(shooter, hopper, intake));

                new JoystickButton(driveController, XboxController.Button.kA.value)
                                .whenHeld(new RangedAutoShoot(shooter, hopper, intake));

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
                new JoystickButton(driveController, XboxController.Button.kLeftBumper.value)
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
        }

        public void teleopDrive() {
                drivetrain.setTeleopRampRates();
                drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.curveDrive(OI.getTriggers(driveController),
                                OI.getLeftStick(driveController), driveController.getXButton()), drivetrain));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void updateRobotPose() {
                field.setRobotPose(drivetrain.getPose());
        }

        public void setFieldTrajectory() {
                Trajectory concatTrajectory = new Trajectory();
                for (RamsetePath p : autoSelector.pathArrayMap.get(autoChooser.getSelected())) {
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
