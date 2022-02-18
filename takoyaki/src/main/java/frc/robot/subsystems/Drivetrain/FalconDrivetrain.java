// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import java.util.LinkedList;
import java.util.Queue;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import com.cuforge.libcu.Lasershark;

public class FalconDrivetrain extends Drivetrain {
  private final WPI_TalonFX frontLeft;
  private final WPI_TalonFX backLeft;
  private final WPI_TalonFX frontRight;
  private final WPI_TalonFX backRight;
  private final Lasershark shark;
  private Queue<Double> distances;

  private final DifferentialDrive diffDrive;

  public FalconDrivetrain() {
    // instantiate motors
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

    setToBrakeMode();

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
    shark = new Lasershark(Constants.kDrive.LASERSHARK_PORT);
    distances = new LinkedList<Double>();
    for( int i = 0; i<20; i++) {
      distances.add(0.0);
    }
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
    distances.add(shark.getDistanceCentimeters());
    distances.remove();
  }

  public Double getDistance() {
    double sum = 0;
    for( Double d : distances) {
      sum += d;
    }
    return sum/7.0;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}