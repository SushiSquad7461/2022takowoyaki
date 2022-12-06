// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase {
  public abstract void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn);

  public abstract void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn, boolean slowMode);

  public abstract void setStatorLimits();

  public abstract void setBrake();

  public abstract void setCoast();

  public abstract void periodic();

  public abstract void simulationPeriodic();

  // get current robot position
  public abstract Pose2d getPose();

  // return current wheel speeds
  public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();

  // reset odometry to given pose
  public abstract void resetOdometry(Pose2d pose);

  public abstract void setOdometry(Trajectory traj);

  // zero encoders
  public abstract void resetEncoders();

  public abstract void tankDriveVolts(double leftVolts, double rightVolts);

  public abstract double getAverageEncoderDistance();

  // scales maximum Drivetrain speed (0 to 1.0)
  public abstract void setMaxOutput(double maxOutput);

  // zero navx (doesn't work consistently so we avoid using)
  public abstract void zeroHeading();

  // return heading in degrees (-180 to 180)
  public abstract double getHeading();

  // return turn rate deg/sec
  public abstract double getTurnRate();

  public abstract void invertDrive();
}