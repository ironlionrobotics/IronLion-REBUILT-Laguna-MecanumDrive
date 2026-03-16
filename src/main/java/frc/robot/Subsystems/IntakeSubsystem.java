package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_NeoIntake = new SparkMax(8, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_IntakeElevar = new SparkMax(5, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   
    private final SparkClosedLoopController m_armIntakeController; 
    
    public IntakeSubsystem() { 
      SparkBaseConfig intakeConfig = new SparkMaxConfig();
      intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
      intakeConfig.inverted(true);
      
      
      SparkBaseConfig intakeElevarConfig = new SparkMaxConfig();
      intakeElevarConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
      intakeElevarConfig.inverted(false);
      intakeElevarConfig.closedLoop.p(0.01).i(0).d(0);
      intakeElevarConfig.encoder.positionConversionFactor(360); 

      m_NeoIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_IntakeElevar.configure(intakeElevarConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
      m_IntakeElevar.getEncoder().setPosition(120);
      m_armIntakeController = m_IntakeElevar.getClosedLoopController();
    }
    public void runIntake() {
        m_NeoIntake.set(0.5);
    }

    public void stopIntake() {
        m_NeoIntake.set(0);
    }

    public void setIntakeElevarAngle(double angle) {
      m_armIntakeController.setSetpoint(angle, ControlType.kPosition);
    }
    public Command setArmAngleCommand(double targetDegrees) {
        return this.run(() -> setIntakeElevarAngle(targetDegrees));
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