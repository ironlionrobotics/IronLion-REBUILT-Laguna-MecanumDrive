package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

import java.lang.reflect.Type;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public static final SparkMax m_NeoIntake = new SparkMax(8, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_IntakeElevar = new SparkMax(5, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   

    
    public IntakeSubsystem() {
      m_IntakeElevar.getEncoder().setPosition(0)
      m_IntakeElevar.setSetpoint(setPoint, ControlType.kPosition);    
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

    public Command runIntakeCommand() {
        return this.run(this::runIntake);
      }

    public Command stopIntakeCommand() {
        return this.run(this::stopIntake);
      }

      // Nuevo comando útil: Parar
    public Command runIntakeElevarCommand() {
        return this.runOnce(this::runIntakeElevar);
      }

    public Command stopIntakeElevarCommand() {
        return this.runOnce(this::stopIntakeElevar);
      }

    public void periodic() {
        
    }
}