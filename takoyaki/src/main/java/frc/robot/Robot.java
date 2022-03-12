// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  boolean hasRun = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateRobotPose();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setDriveBrake();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.setFieldTrajectory();
    if (!hasRun) {
      m_robotContainer.setInitialPose();
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_robotContainer.autoDrive();
    hasRun = true;
    m_robotContainer.setDriveBrake();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.setDriveBrake();
    m_robotContainer.teleopDrive();
    m_robotContainer.drivetrain.configOpenloopRamp(Constants.kDrive.OPEN_LOOP_RAMP_RATE);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // m_robotContainer.tankDriveVolts(2, 2);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
