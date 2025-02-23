package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    SparkMax m_leftShooter = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    SparkMax m_rightShooter = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    public void shoot(double speed){
        m_leftShooter.set(speed);
        m_rightShooter.set(-speed);
    }

    public void trough(double speed){
        m_leftShooter.set(speed);
        m_rightShooter.set(-speed*0.2);
    }
}
