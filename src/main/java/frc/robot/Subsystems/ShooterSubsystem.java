package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_NEObeltIndexer = new SparkMax(9, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOshooter = new SparkMax(11, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_NEOfeeder = new SparkMax(10, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkClosedLoopController m_shooterController; 

    public ShooterSubsystem() {
        SparkBaseConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        shooterConfig.inverted(false);
        shooterConfig.closedLoop.p(0.01).i(0).d(0);
        //TODO: ask robert for insight in the relation of gears at the shooter 
        shooterConfig.encoder.velocityConversionFactor(1); // factor 1 porque el controlador de velocidad de SparkMax ya maneja las conversiones internamente para RPM
        m_shooterController = m_NEOshooter.getClosedLoopController();

        SparkBaseConfig feederAndIndexerConfig = new SparkMaxConfig();
        feederAndIndexerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        feederAndIndexerConfig.inverted(false);
        
        m_NEOshooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_NEObeltIndexer.configure(feederAndIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_NEOfeeder.configure(feederAndIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void setShooterRPM(double rpm) {
        m_shooterController.setSetpoint(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }
    public Command setShooterRPMCommand(double RPM) {
        return this.run(() -> setShooterRPM(RPM));
    }
    public void runShooter() {
        m_NEOshooter.set(.4);
    }

    public void stopShooter() {
        m_NEOshooter.stopMotor();
    }

    public void runIndexerAndFeeder() {
        m_NEObeltIndexer.set(.5);
        m_NEOfeeder.set(.5);
    }

    public void stopIndexerAndFeeder() {
        m_NEObeltIndexer.stopMotor();
        m_NEOfeeder.stopMotor();
    }

    public Command runShooterCommand() {
        return this.run(this::runShooter);
      }

    public Command stopShooterCommand() {
        return this.run(this::stopShooter);
      }

      // Nuevo comando útil: Parar
    public Command runIndexerAndFeederCommand() {
        return this.runOnce(this::runIndexerAndFeeder);
      }

    public Command stopIndexerAndFeederCommand() {
        return this.runOnce(this::stopIndexerAndFeeder);
      }

}
