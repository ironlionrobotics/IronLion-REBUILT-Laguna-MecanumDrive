package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public static final SparkMax m_NEObeltIndexer = new SparkMax(9, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_NEOshooter = new SparkMax(11, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_NEOfeeder = new SparkMax(5, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    public ShooterSubsystem() {

    }
    public void runShooter() {
        m_NEOshooter.set(1);
    }

    public void stopShooter() {
        m_NEOshooter.stopMotor();
    }

    public void runBeltIndexer() {
        m_NEObeltIndexer.set(.5);
    }

    public void stopBeltIndexer() {
        m_NEObeltIndexer.stopMotor();
    }

    public void runFeeder() {
        m_NEOfeeder.set(.5);
    }

    public void stopFeeder() {
        m_NEOfeeder.stopMotor();
    }
}
