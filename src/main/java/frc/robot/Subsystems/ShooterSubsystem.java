package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunableConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

/**
 * Controla el sistema de disparo e indexación de piezas (FUEL).
 * Incluye llantas expulsoras (Shooter), la faja interna que guarda múltiples piezas (Belt Indexer) 
 * y el rodillo veloz terminal que empuja abruptamente la pieza hacia el shooter (Feeder).
 */
public class ShooterSubsystem extends SubsystemBase {
    // Controladores inerciales
    private final SparkMax m_NEObeltIndexer = new SparkMax(ShooterConstants.kBeltIndexerPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOshooter = new SparkMax(ShooterConstants.kShooterPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOfeeder = new SparkMax(ShooterConstants.kFeederPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    // InterpolatingTreeMap (Inteligencia Predictiva): Nos permite interpolar (adivinar valores intermedios matemáticamente) 
    // basándonos en si estamos a X metros de distancia para dar Y velocidad en RPM al sistema aéreo.
    private final InterpolatingDoubleTreeMap m_shooterMap = new InterpolatingDoubleTreeMap();
    
    // Controlador PID de hardware. Se corre con alta tasa de actualización a nivel físico de tarjeta electrónica en vez de WiFi.
    private final SparkClosedLoopController m_shooterPID;
    
    public ShooterSubsystem() {
        com.revrobotics.spark.config.SparkMaxConfig shooterConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        // CUIDADO FÍSICO: El Shooter necesita IdleMode Coast (rodaje muerto) obligatoriamente
        // porque si usara Brake (freno seco), la inercia a 4000 RPM de sus llantas pesadas romperá
        // los tornillos del eje al intentar frenar en 0 segundos cuando soltemos el Trigger.
        shooterConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
                     .smartCurrentLimit(ShooterConstants.kShooterCurrentLimit); // Evita jalar baterías completas
      
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

        // Tabla de Calibración cargada desde Constants.ShooterConstants.kShooterCalibration
        // Para añadir puntos de calibración, editar SOLO Constants.java
        for (double[] entry : ShooterConstants.kShooterCalibration) {
            m_shooterMap.put(entry[0], entry[1]);
        }
    }

    /** Prende las llantas de disparo. Usa TunableConstants para poder ajustar en vivo. */
    public void runShooter() {
        m_NEOshooter.set(TunableConstants.shooterSpeed);
    }

    public void stopShooter() {
        m_NEOshooter.stopMotor(); 
    }

    /**
     * Modo Adaptativo Magistral: 
     * Dicta un PID interno en ControlType kVelocity pasándole el target de RPM interpolados desde la regresión matemática.
     */
    public void runShooterAtDistance(double distance) {
        double targetRPM = m_shooterMap.get(distance);
        m_shooterPID.setReference(targetRPM, ControlType.kVelocity);
    }
    
    /** Regresa true cuando las llantas alcanzaron la velocidad deseada (tolerancia configurable desde Constants) */
    public boolean isShooterAtSpeed(double targetDistance) {
        double targetRPM = m_shooterMap.get(targetDistance);
        return Math.abs(m_NEOshooter.getEncoder().getVelocity() - targetRPM) < TunableConstants.shooterRPMTolerance;
    }

    // Acciones Manuales de Alimentación de Fuel Interno
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

    // =========================================================
    // COMANDOS DE ACCIÓN BINDING (Para Botones de Xbox/PathPlanner)
    // =========================================================
    // runEnd garantiza que todos regresen al estado 0 V apagados al morir el comando.

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

    /**
     * Agrupa toda la purga del cañón a funcionar y apagarse unísono. Especial para Nivel Hub L1 de frente.
     */
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
