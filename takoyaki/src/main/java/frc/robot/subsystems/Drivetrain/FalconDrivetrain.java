// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ramsete.PathPlannerPath;

public class FalconDrivetrain extends Drivetrain {
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
  private int inverted; // 1 is forward, -1 is reverse

  // private final DifferentialDrive diffDrive;

  public FalconDrivetrain() {

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

    setBrake();

    /* set up followers */
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    backLeft.setInverted(TalonFXInvertType.CounterClockwise);
    frontRight.setInverted(TalonFXInvertType.Clockwise);
    backRight.setInverted(TalonFXInvertType.Clockwise);

    frontLeft.configSupplyCurrentLimit(Constants.currentLimit(40));
    backLeft.configSupplyCurrentLimit(Constants.currentLimit(40));
    frontRight.configSupplyCurrentLimit(Constants.currentLimit(40));
    backRight.configSupplyCurrentLimit(Constants.currentLimit(40));

    // frontLeft.configOpenloopRamp(0.65);
    // backLeft.configOpenloopRamp(0.65);
    // frontRight.configOpenloopRamp(0.65);
    // backRight.configOpenloopRamp(0.65);

    this.configOpenloopRamp(0);

    // frontLeft.configClosedloopRamp(Constants.kDrive.CLOSED_LOOP_RAMP_RATE);
    // backLeft.configClosedloopRamp(Constants.kDrive.CLOSED_LOOP_RAMP_RATE);
    // frontRight.configClosedloopRamp(Constants.kDrive.CLOSED_LOOP_RAMP_RATE);
    // backRight.configClosedloopRamp(Constants.kDrive.CLOSED_LOOP_RAMP_RATE);

    inverted = 1;

    // config odometry and auto values
    nav = new AHRS(SPI.Port.kMXP);
    nav.enableBoardlevelYawReset(false);

    zeroOffset = 0;
    angleOffset = 0;
    navZeroed = false;

    odometry = new DifferentialDriveOdometry(nav.getRotation2d());

  }

  double lastTriggerSpeed = 0;

  public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn) {
    double val = linearVelocity;
    if (val != lastTriggerSpeed) {
      if (val < lastTriggerSpeed) {
        val = lastTriggerSpeed - Constants.kDrive.TRIGGER_SPEED_DERIVATIVE;
      } else {
        val = lastTriggerSpeed + Constants.kDrive.TRIGGER_SPEED_DERIVATIVE;
      }
      lastTriggerSpeed = val;
    }
    double sensorVelocity = Constants.convertTransToRPM(frontLeft.getSelectedSensorVelocity());
    if (Math.abs(sensorVelocity) < Constants.kDrive.MINIMUM_SENSOR_VELOCITY && linearVelocity != 0) {
      val = Constants.kDrive.LINEAR_SCALING_MIN_SPEED * (val > 0 ? 1 : -1);
    }
    if (isQuickturn) {
      angularVelocity /= Constants.kDrive.QUICK_TURN_DAMPENER;
    }
    diffDrive.curvatureDrive(val * inverted, angularVelocity * inverted, isQuickturn);
  }

  public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn, boolean slowMode) {
    if (slowMode) {
      curveDrive(Constants.kDrive.SLOW_MODE_VELOCITY, angularVelocity, isQuickturn);
    } else {
      curveDrive(linearVelocity, angularVelocity, isQuickturn);
    }

  }

  public void breakInGearboxes() {
    double speed = 0.001;
    double motorPower = Math.sin(System.currentTimeMillis() * speed);
    // diffDrive.curvatureDrive(motorPower, 1, false);
    frontRight.set(motorPower);
  }

  public void invertDrive() {
    inverted *= -1;
  }

  @Override
  public void periodic() {
    // find IMU zero offset after calibration
    if (!nav.isCalibrating() && !navZeroed) {
      zeroOffset = getHeading();
      navZeroed = true;
    }

    odometry.update(new Rotation2d(Math.toRadians(-(getHeading() - angleOffset))),
        frontLeft.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS,
        frontRight.getSelectedSensorPosition() * Constants.kDrive.TICKS_TO_METERS);

    SmartDashboard.putNumber("gyro pitch", nav.getPitch());
  }

  @Override
  public void simulationPeriodic() {
  }

  // get current robot position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void configOpenloopRamp(double openLoopRamp) {
    frontLeft.configOpenloopRamp(openLoopRamp);
    backLeft.configOpenloopRamp(openLoopRamp);
    frontRight.configOpenloopRamp(openLoopRamp);
    backRight.configOpenloopRamp(openLoopRamp);
  }

  // return current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        frontLeft.getSelectedSensorVelocity(),
        frontRight.getSelectedSensorVelocity());
  }

  // reset odometry to given pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(-(getHeading() - zeroOffset)));
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
    return -nav.getYaw();
    // note: getAngle returns accumulated yaw (can be <0 or >360)
    // getYaw has a 360 degree period
    /*
     * if (nav.getYaw() <= 0) {
     * return nav.getYaw() + 180;
     * } else {
     * return nav.getYaw() - 180;
     * }
     */
  }

  // return turn rate deg/sec
  public double getTurnRate() {
    // negative since navx's positive direction is opposite of the expected/wpilib
    // standard
    return -nav.getRate();
  }

  // set ramp rates for teleop
  public void setTeleopRampRates() {
    frontLeft.configOpenloopRamp(Constants.kDrive.OPEN_LOOP_RAMP_RATE);
    frontRight.configOpenloopRamp(Constants.kDrive.OPEN_LOOP_RAMP_RATE);
    backLeft.configOpenloopRamp(Constants.kDrive.OPEN_LOOP_RAMP_RATE);
    backRight.configOpenloopRamp(Constants.kDrive.OPEN_LOOP_RAMP_RATE);
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