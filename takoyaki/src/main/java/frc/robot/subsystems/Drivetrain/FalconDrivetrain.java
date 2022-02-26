// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class FalconDrivetrain extends Drivetrain {
  private final WPI_TalonFX frontLeft;
  private final WPI_TalonFX backLeft;
  private final WPI_TalonFX frontRight;
  private final WPI_TalonFX backRight;

  private final DifferentialDrive diffDrive;

  public FalconDrivetrain() {
    // instantiate motors
    frontLeft = new WPI_TalonFX(4);
    backLeft = new WPI_TalonFX(3);
    frontRight = new WPI_TalonFX(15);
    backRight = new WPI_TalonFX(16);

    // instantiate differential drive
    diffDrive = new DifferentialDrive(frontLeft, frontRight);

    /* factory default values */
    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

    setToBrakeMode();

    /* set up followers */
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    backLeft.setInverted(TalonFXInvertType.CounterClockwise);
    frontRight.setInverted(TalonFXInvertType.Clockwise);
    backRight.setInverted(TalonFXInvertType.Clockwise);

    frontLeft.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */
  }

  public void setToBrakeMode() {
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setToCoastMode() {
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
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
    SmartDashboard.putNumber("fl", frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("bl", backLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("fr", frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("br", backRight.getSelectedSensorPosition());

    
    SmartDashboard.putNumber("fl vel", frontLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("bl vel", backLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("fr vel", frontRight.getSelectedSensorVelocity());
    SmartDashboard.putNumber("br vel", backRight.getSelectedSensorVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}