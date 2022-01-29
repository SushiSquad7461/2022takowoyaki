// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX frontRight;
  private WPI_TalonFX backRight;
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backLeft;

  private DifferentialDrive diffDrive;

  public Drivetrain() {

    frontRight = new WPI_TalonFX(Constants.kDrive.FRONT_RIGHT_ID);
    backRight = new WPI_TalonFX(Constants.kDrive.BACK_RIGHT_ID);
    frontLeft = new WPI_TalonFX(Constants.kDrive.FRONT_LEFT_ID);
    backLeft = new WPI_TalonFX(Constants.kDrive.BACK_LEFT_ID);

    /* factory default values */
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();
    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();

    /* set up followers */
    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontRight.setInverted(TalonFXInvertType.CounterClockwise);
    frontLeft.setInverted(TalonFXInvertType.Clockwise);
    backRight.setInverted(TalonFXInvertType.CounterClockwise);
    backLeft.setInverted(TalonFXInvertType.Clockwise);

    frontRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);

    
    diffDrive = new DifferentialDrive(frontLeft, frontRight);
    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */ 
  }

  public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn) {
    if (isQuickturn) {
      angularVelocity /= 3;
    }
    diffDrive.curvatureDrive(linearVelocity, angularVelocity, isQuickturn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}