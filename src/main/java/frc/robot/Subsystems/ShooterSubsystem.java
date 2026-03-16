package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_NEObeltIndexer = new SparkMax(ShooterConstants.kBeltIndexerPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOshooter = new SparkMax(ShooterConstants.kShooterPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOfeeder = new SparkMax(ShooterConstants.kFeederPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    private final InterpolatingDoubleTreeMap m_shooterMap = new InterpolatingDoubleTreeMap();
    private final SparkClosedLoopController m_shooterPID;
    
    public ShooterSubsystem() {
        com.revrobotics.spark.config.SparkMaxConfig shooterConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        shooterConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
                     .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit);
      
        com.revrobotics.spark.config.SparkMaxConfig indexerConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        indexerConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
                     .smartCurrentLimit(ShooterConstants.kBeltIndexerCurrentLimit);
      
        com.revrobotics.spark.config.SparkMaxConfig feederConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        feederConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
                    .smartCurrentLimit(ShooterConstants.kFeederCurrentLimit);

        m_NEOshooter.configure(shooterConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        m_NEObeltIndexer.configure(indexerConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        m_NEOfeeder.configure(feederConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

        m_shooterPID = m_NEOshooter.getClosedLoopController();

        // Tabla de Interpolación: Distancia (ty o metros) -> RPM
        // Estos valores deben calibrarse en el taller.
        m_shooterMap.put(-10.0, 1500.0); // Lejos
        m_shooterMap.put(0.0, 3000.0);   // Medio
        m_shooterMap.put(10.0, 4500.0);  // Cerca
    }

    public void runShooter() {
        m_NEOshooter.set(ShooterConstants.kShooterSpeed);
    }

    public void stopShooter() {
        m_NEOshooter.stopMotor();
    }

    /**
     * Ajusta la velocidad del shooter basándose en la distancia provista por la tabla.
     */
    public void runShooterAtDistance(double distance) {
        double targetRPM = m_shooterMap.get(distance);
        m_shooterPID.setReference(targetRPM, ControlType.kVelocity);
    }
    
    public boolean isShooterAtSpeed(double targetDistance) {
        double targetRPM = m_shooterMap.get(targetDistance);
        return Math.abs(m_NEOshooter.getEncoder().getVelocity() - targetRPM) < 100;
    }

    public void runBeltIndexer() {
        m_NEObeltIndexer.set(ShooterConstants.kBeltIndexerSpeed);
    }

    public void stopBeltIndexer() {
        m_NEObeltIndexer.stopMotor();
    }

    public void runFeeder() {
        m_NEOfeeder.set(ShooterConstants.kFeederSpeed);
    }

    public void stopFeeder() {
        m_NEOfeeder.stopMotor();
    }

    public Command runShooterCommand() {
        return this.runEnd(this::runShooter, this::stopShooter);
    }

    public Command stopShooterCommand() {
        return this.runOnce(this::stopShooter);
    }

    public Command runBeltIndexerCommand() {
        return this.runEnd(this::runBeltIndexer, this::stopBeltIndexer);
    }

    public Command stopBeltIndexerCommand() {
        return this.runOnce(this::stopBeltIndexer);
    }

    public Command runFeederCommand() {
        return this.runEnd(this::runFeeder, this::stopFeeder);
    }

    public Command stopFeederCommand() {
        return this.runOnce(this::stopFeeder);
    }

    public Command runFullShooterSystemCommand() {
        return this.runEnd(() -> {
            runShooter();
            runBeltIndexer();
            runFeeder();
        }, () -> {
            stopShooter();
            stopBeltIndexer();
            stopFeeder();
        });
    }

    public SparkMax getBeltIndexer() { return m_NEObeltIndexer; }
    public SparkMax getShooter() { return m_NEOshooter; }
    public SparkMax getFeeder() { return m_NEOfeeder; }
}
