package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.Constants.DriveConstants;

public class ClimberSubsystem extends SubsystemBase { 
    
    private final SparkMax m_neoClimber = new SparkMax(DriveConstants.kNeoClimberPort, MotorType.kBrushless); 
    
    public ClimberSubsystem() {
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.encoder.positionConversionFactor(15.0); 

        climberConfig
            .idleMode(IdleMode.kBrake) 
            .inverted(false)
            .smartCurrentLimit(40);
        
        climberConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .forwardSoftLimit(60.0) 
            .reverseSoftLimitEnabled(false)
            .reverseSoftLimit(0.0); 

        m_neoClimber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runClimber() {
        m_neoClimber.set(0.1);
    }
    
    public void runClimberReverse() {
        m_neoClimber.set(-0.1);
    }
    
    public void stopClimber() {
        m_neoClimber.stopMotor();
    }

    public Command runClimberCommand() {
        return this.run(this::runClimber);
    }

    public Command runClimberReverseCommand() {
        return this.run(this::runClimberReverse);
    }
    
    public Command stopClimberCommand() {
        return this.run(this::stopClimber);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spark Max Climber position", m_neoClimber.getEncoder().getPosition());
    }
}