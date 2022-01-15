// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;



public class Climb extends SubsystemBase {

    WPI_TalonFX left = new WPI_TalonFX(Constants.climbLeftCAN);
    WPI_TalonFX right = new WPI_TalonFX(Constants.climbRightCAN);

    Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidChannel);

    boolean brake = false;

  public Climb() {
    left.configFactoryDefault();
    right.configFactoryDefault();

    left.follow(right);

    left.setInverted(TalonFXInvertType.OpposeMaster);
    right.setInverted(TalonFXInvertType.Clockwise);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
  }

  public void raiseClimbStart(){
    right.set(ControlMode.PercentOutput, Constants.climbMaxUp);
  }
  public void lowerClimbStart(){
    right.set(ControlMode.PercentOutput, Constants.climbMaxDown);
  }
  public void stopClimb(){
    right.set(ControlMode.PercentOutput, 0);
  }
  public void triggerBrake(){
    brakeSolenoid.set(true);
  }
  public void releaseBrake(){
      brakeSolenoid.set(false);
  }
}
