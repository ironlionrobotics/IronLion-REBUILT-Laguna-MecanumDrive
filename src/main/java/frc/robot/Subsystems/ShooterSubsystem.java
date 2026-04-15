package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SafetyConstants;
import frc.robot.Constants.TunableConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_NEObeltIndexer = new SparkMax(DriveConstants.kNeoBeltIndexerPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOfeeder = new SparkMax(DriveConstants.kNeoFeederPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOshooter = new SparkMax(DriveConstants.kNeoShooterPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    public ShooterSubsystem() {
        SparkBaseConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        shooterConfig.inverted(true);
        shooterConfig.smartCurrentLimit((int) SafetyConstants.kMaxMotorCurrentAmps); 
        
        SparkBaseConfig feederAndIndexerConfig = new SparkMaxConfig();
        feederAndIndexerConfig
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit((int) SafetyConstants.kMaxMotorCurrentAmps);

        m_NEOshooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_NEObeltIndexer.configure(feederAndIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_NEOfeeder.configure(feederAndIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runShooter() { m_NEOshooter.set(TunableConstants.shooterSpeed); }
    public void runShooterReverse() { m_NEOshooter.set(-TunableConstants.shooterSpeed); }
    public void stopShooter() { m_NEOshooter.stopMotor(); }

    public void runIndexerAndFeeder() {
        m_NEObeltIndexer.set(TunableConstants.beltIndexerSpeed);
        m_NEOfeeder.set(TunableConstants.feederSpeed);
    }

    public void runIndexerAndFeederReverse() {
        m_NEObeltIndexer.set(TunableConstants.beltIndexerReverseSpeed);
        m_NEOfeeder.set(TunableConstants.feederReverseSpeed);
    }

    public void stopIndexerAndFeeder() {
        m_NEObeltIndexer.stopMotor();
        m_NEOfeeder.stopMotor();
    }

    // Command Factories
    public Command stopShooterCommand() { return this.runOnce(this::stopShooter); }
    public Command runIndexerAndFeederCommand() { return this.runOnce(this::runIndexerAndFeeder); }
    public Command runIndexerAndFeederReverseCommand() { return this.runOnce(this::runIndexerAndFeederReverse); }
    public Command stopIndexerAndFeederCommand() { return this.runOnce(this::stopIndexerAndFeeder); }
    public Command runShooterCommand() { return this.runOnce(this::runShooter); }
    public Command runShooterReverseCommand() { return this.runOnce(this::runShooterReverse); }

    // Telemetry Getters
    public double getAppliedOutput() { return m_NEOshooter.getAppliedOutput(); }
    public double getShooterTemp() { return m_NEOshooter.getMotorTemperature(); }
    public double getShooterCurrent() { return m_NEOshooter.getOutputCurrent(); }

    @Override
    public void periodic() {
    }
}