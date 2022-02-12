// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX frontLeft;
  private final WPI_TalonFX backLeft;
  private final WPI_TalonFX frontRight;
  private final WPI_TalonFX backRight;

  private final DifferentialDrive diffDrive;

  // odometry and autonomous
  private final DifferentialDriveOdometry odometry;
  private final AHRS nav;
  private double zeroOffset;
  private double angleOffset;
  boolean navZeroed;

  public Drivetrain() {

    // config motors
    frontLeft = new WPI_TalonFX(Constants.kDrive.FRONT_LEFT_ID);
    backLeft = new WPI_TalonFX(Constants.kDrive.BACK_LEFT_ID);
    frontRight = new WPI_TalonFX(Constants.kDrive.FRONT_RIGHT_ID);
    backRight = new WPI_TalonFX(Constants.kDrive.BACK_RIGHT_ID);

    // instantiate differential drive
    diffDrive = new DifferentialDrive(frontLeft, frontRight);

    /* factory default values */
    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

    /* set up followers */
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    backLeft.setInverted(TalonFXInvertType.CounterClockwise);
    frontRight.setInverted(TalonFXInvertType.Clockwise);
    backRight.setInverted(TalonFXInvertType.Clockwise);
    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */ 

    // config odometry and auto values
    nav = new AHRS(SPI.Port.kMXP);
    nav.enableBoardlevelYawReset(false);

    zeroOffset = 0;
    angleOffset = 0;
    navZeroed = false;

    odometry = new DifferentialDriveOdometry(nav.getRotation2d());
  }

  public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn) {
    if (isQuickturn) {
      angularVelocity /= Constants.kDrive.QUICK_TURN_DAMPENER;
    }
    diffDrive.curvatureDrive(linearVelocity, angularVelocity, isQuickturn);
  }

  @Override
  public void periodic() {
    // find IMU zero offset after calibration
    if (!nav.isCalibrating() && !navZeroed) {
      zeroOffset = nav.getYaw();
      navZeroed = true;
    }

    odometry.update(new Rotation2d(Math.toRadians(-(nav.getYaw() - angleOffset))), 
    frontLeft.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS,
    frontRight.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS);
  
    SmartDashboard.putNumber("heading", -(nav.getYaw() - zeroOffset));
    SmartDashboard.putNumber("odometry x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry y", odometry.getPoseMeters().getY());
  }

  @Override
  public void simulationPeriodic() {}

  // get current robot position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // return current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( 
      frontLeft.getSelectedSensorVelocity(), 
      frontRight.getSelectedSensorVelocity()
    );
  }

  // reset odometry to given pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(-(nav.getYaw() - zeroOffset)));
  }

  public void setOdometry(Trajectory traj) {
    resetOdometry(new Pose2d());
    angleOffset = traj.getInitialPose().getRotation().getDegrees() + zeroOffset;
    odometry.resetPosition(traj.getInitialPose(), traj.getInitialPose().getRotation());
  }

  // zero encoders
  public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (frontLeft.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition()) / 2.0;
  }

  // scales maximum Drivetrain speed (0 to 1.0)
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

  // zero navx (doesn't work consistently so we avoid using)
  public void zeroHeading() {
    nav.reset();
  }

  // return heading in degrees (-180 to 180)
  public double getHeading() {
    return nav.getYaw();
    // note: getAngle returns accumulated yaw (can be <0 or >360)
    //   getYaw has a 360 degree period
  }

  // return turn rate deg/sec
  public double getTurnRate() {
    // negative since navx's positive direction is opposite of the expected/wpilib standard
    return -nav.getRate();
  }

  // set motors to brake mode
  public void setBrake() {
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }

  // set motors to brake mode
  public void setCoast() {
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
  }
  
}