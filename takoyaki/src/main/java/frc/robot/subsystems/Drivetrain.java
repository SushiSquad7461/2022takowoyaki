// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  WPI_TalonFX rightFront = new WPI_TalonFX(Constants.kDrive.FRONT_RIGHT_ID);
  WPI_TalonFX rightBack = new WPI_TalonFX(Constants.kDrive.BACK_RIGHT_ID);
  WPI_TalonFX leftFront = new WPI_TalonFX(Constants.kDrive.FRONT_LEFT_ID);
  WPI_TalonFX leftBack = new WPI_TalonFX(Constants.kDrive.BACK_LEFT_ID);

  DifferentialDrive diffDrive = new DifferentialDrive(leftFront, rightFront);

  private int angleInvert;

  public Drivetrain() {
    /* factory default values */
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();

    /* set up followers */
    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    rightFront.setInverted(TalonFXInvertType.CounterClockwise);
    leftFront.setInverted(TalonFXInvertType.Clockwise);
    rightBack.setInverted(TalonFXInvertType.CounterClockwise);
    leftBack.setInverted(TalonFXInvertType.Clockwise);

    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */ 
  }

  public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickturn) {
    if (isQuickturn) {
      angularVelocity /= 2;
    }
    diffDrive.curvatureDrive(linearVelocity, angularVelocity * angleInvert, isQuickturn);
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