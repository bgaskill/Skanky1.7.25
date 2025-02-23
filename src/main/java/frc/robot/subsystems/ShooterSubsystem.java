package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class ShooterSubsystem extends SubsystemBase{
    SparkMax m_leftShooter = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    SparkMax m_rightShooter = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    LaserCan lc = new LaserCan(38);
    LaserCan lc2 = new LaserCan(39);

    public void shoot(double speed){
        m_leftShooter.set(speed);
        m_rightShooter.set(-speed);
    }

    public void laserIntake(double speed){
        m_leftShooter.set(speed);
        m_rightShooter.set(-speed);


        LaserCan.Measurement measurement = lc.getMeasurement();
        LaserCan.Measurement measurement2 = lc2.getMeasurement();

        SmartDashboard.putNumber ("Laser 1",measurement.distance_mm);
        SmartDashboard.putNumber ("Laser 2",measurement2.distance_mm);

        if (measurement != null && measurement2 != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && measurement2.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT 
        && measurement.distance_mm < 100
        && measurement2.distance_mm > 100
        )

        {
            m_leftShooter.set(0);
            m_rightShooter.set(0);

        }
    }

    public void reverse(double speed){
        m_leftShooter.set(-speed);
        m_rightShooter.set(speed);
    }


    public void trough(double speed){
        m_leftShooter.set(speed);
        m_rightShooter.set(-speed*0.2);
    }
}
