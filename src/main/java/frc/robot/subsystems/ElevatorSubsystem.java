// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class ElevatorSubsystem extends SubsystemBase {
  
  private final TalonFX talonElevator = new TalonFX (ElevatorConstants.kLeftElevatorCanId);
  private final TalonFX talonElevator2 = new TalonFX(ElevatorConstants.kRightElevatorCanId);
  
  
  
  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    talonElevator2.setControl(new StrictFollower(ElevatorConstants.kLeftElevatorCanId));
    
    var slot0Configs =new Slot0Configs();
    slot0Configs.kP= 2.4;
    slot0Configs.kI= 0;
    slot0Configs.kD= .1;
    slot0Configs.kS= .1;   //static friction compensation
    slot0Configs.kV= .12;  //velocity feedforwar
    slot0Configs.kA= .01;   //Acceleration feedforward
    
    var talonElevatorConfig =new TalonFXConfiguration();
    var ElevatorMM = talonElevatorConfig.MotionMagic;
    ElevatorMM.MotionMagicCruiseVelocity = 200;
    ElevatorMM.MotionMagicAcceleration = 160;
    ElevatorMM.MotionMagicJerk = 1160;
    
    


    talonElevator.getConfigurator().apply(slot0Configs);

    talonElevator.getConfigurator().apply(ElevatorMM);

    //final MotionMagicVoltage m_level1 = new MotionMagicVoltage(5);
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorControl(double speed){

    talonElevator.set(-speed);
    //talonElevator2.set(-speed);
}
public void level0Position(double position){

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  talonElevator.setControl(m_request.withPosition(position));
}
public void level1Position(double position){

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  talonElevator.setControl(m_request.withPosition(position));

}
public void level2Position(double position){

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  talonElevator.setControl(m_request.withPosition(position));

}


}
