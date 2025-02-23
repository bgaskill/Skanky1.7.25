// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.StrictFollower;



public class ElevatorSubsystem extends SubsystemBase {
  
  private final TalonFX talonElevator = new TalonFX (ElevatorConstants.kLeftElevatorCanId);
  private final TalonFX talonElevator2 = new TalonFX(ElevatorConstants.kRightElevatorCanId);

  
  /** Creates a new Elevator. */
  public ElevatorSubsystem() {

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorControl(double speed){

    talonElevator.set(-speed);
    talonElevator2.set(-speed);
}
}
