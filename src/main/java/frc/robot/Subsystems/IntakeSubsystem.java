package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {

    public static final SparkMax m_NeoIntake = new SparkMax(DriveConstants.kNeoIntakePort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_IntakeElevar = new SparkMax(DriveConstants.KIntakeElevarPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   

    public IntakeSubsystem() {
        
    }
    public void runIntake() {
        m_NeoIntake.set(0.5);
    }

    public void stopIntake() {
        m_NeoIntake.set(0);
    }

    public void runIntakeElevar() {
        m_IntakeElevar.set(0.5);
    }

    public void stopIntakeElevar() {
        m_IntakeElevar.set(0);
    }

    public void periodic() {
        
    }
}