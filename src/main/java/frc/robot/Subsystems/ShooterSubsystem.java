package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunableConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_NEObeltIndexer = new SparkMax(DriveConstants.kNeoBeltIndexerPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOfeeder = new SparkMax(DriveConstants.kNeoFeederPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOshooter = new SparkMax(DriveConstants.kNeoShooterPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkClosedLoopController m_shooterController; 
    
    private double m_targetRPM = 0.0; 

    public ShooterSubsystem() {
        SparkBaseConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        shooterConfig.inverted(false);
        shooterConfig.closedLoop
            .p(0.0001)
            .i(0)
            .d(0);
        
        shooterConfig.closedLoop.feedForward
            .kV(1.0 / 5676.0);

        shooterConfig.smartCurrentLimit(40); 
        shooterConfig.encoder.velocityConversionFactor(2); 
        
        m_shooterController = m_NEOshooter.getClosedLoopController();
        
        SparkBaseConfig feederAndIndexerConfig = new SparkMaxConfig();
        feederAndIndexerConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .inverted(false);
        
        feederAndIndexerConfig.smartCurrentLimit(30);

        m_NEOshooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_NEObeltIndexer.configure(feederAndIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_NEOfeeder.configure(feederAndIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runShooter() {
        m_NEOshooter.set(TunableConstants.shooterSpeed);
    }

    public void stopShooter() {
        m_targetRPM = 0.0;
        m_NEOshooter.stopMotor();
    }

    public void runIndexerAndFeeder() {
        m_NEObeltIndexer.set(TunableConstants.beltIndexerSpeed);
        m_NEOfeeder.set(TunableConstants.feederSpeed);
    }

    public void stopIndexerAndFeeder() {
        m_NEObeltIndexer.stopMotor();
        m_NEOfeeder.stopMotor();
    }

    public Command stopShooterCommand() {
        return this.run(this::stopShooter);
    }

    public Command runIndexerAndFeederCommand() {
        return this.runOnce(this::runIndexerAndFeeder);
    }

    public Command stopIndexerAndFeederCommand() {
        return this.runOnce(this::stopIndexerAndFeeder);
    }

    public void setShooterRPM(double rpm) {
        m_targetRPM = rpm; 
        m_shooterController.setSetpoint(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public Command setShooterRPMCommand(double RPM) {
        return this.run(() -> setShooterRPM(RPM));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", m_NEOshooter.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter TARGET RPM", m_targetRPM);
    }
}