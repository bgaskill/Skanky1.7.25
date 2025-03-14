// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PokerConstants;

public class PokerSubsystem extends SubsystemBase {

  SparkMax pokeyMotor = new SparkMax(35, MotorType.kBrushless);
  RelativeEncoder pokeyEncoder = pokeyMotor.getEncoder();
  PIDController pokeyPID = new PIDController(0.04, 0, 0);

    /** Creates a new PokerSubsystem. */
  public PokerSubsystem() {}
  
  public void pokeyRotate(double speed){
    double pos = pokeyEncoder.getPosition();
    if (pos > PokerConstants.fullRot){
      pokeyEncoder.setPosition(pos%PokerConstants.fullRot);
    }else if (pos < 0){
      pokeyEncoder.setPosition(PokerConstants.fullRot-pos);
    }
    pokeyMotor.set(speed);
  }

  public void pokeSet(double currentPos, double targetPos){
    pokeyMotor.set(pokeyPID.calculate(pokeyEncoder.getPosition(), targetPos));
  }

  public void pokeZero() { pokeSet(pokeyEncoder.getPosition(), 0); };

  public void pokeLow() { pokeSet(pokeyEncoder.getPosition(), 208.774338); }

  public void pokeHigh() { pokeSet(pokeyEncoder.getPosition(), 183.325134); }

  public void pokeReset() { pokeyEncoder.setPosition(0);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pokey Position", pokeyEncoder.getPosition());
  }
}
