package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    
    private double m_targetRPM = 3000; 

    // Create the Interpolating Tree
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        SparkBaseConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        shooterConfig.inverted(true);
        
        shooterConfig.closedLoop.feedForward
            .kV(1.0 / 11352.0);

        shooterConfig.smartCurrentLimit(30); 
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

        // --- SHOOTER CALIBRATION DATA ---
        // Format: m_rpmMap.put(Limelight_TY_Angle, Target_RPM);
        // INSTRUCTIONS: Tune these numbers physically on the carpet! 
        m_rpmMap.put(15.0, 0.4); // Very close to the speaker
        m_rpmMap.put(5.0, 0.6);  // Mid-range
        m_rpmMap.put(-2.0, 0.8); // Far away
        m_rpmMap.put(-10.0,1.0); // Maximum distance
    }

    /**
     * Feeds the Limelight TY angle into the tree and returns the perfect RPM.
     */
    public double getCalculatedRPM(double limelightTy) {
        return m_rpmMap.get(limelightTy);
    }

    public boolean isAtSpeed() {
        double currentRPM = m_NEOshooter.getEncoder().getVelocity();
        return Math.abs(currentRPM - m_targetRPM) <= TunableConstants.shooterRPMTolerance;
    }

    public void runShooter() {
        m_targetRPM = 0.0;

        m_NEOshooter.set(TunableConstants.shooterSpeed);
    }

    public void stopShooter() {
        m_NEOshooter.stopMotor();
    }

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

    public Command stopShooterCommand() {
        return this.run(this::stopShooter);
    }

    public Command runIndexerAndFeederCommand() {
        return this.runOnce(this::runIndexerAndFeeder);
    }

    public Command runIndexerAndFeederReverseCommand() {
        return this.runOnce(this::runIndexerAndFeederReverse);
    }

    public Command stopIndexerAndFeederCommand() {
        return this.runOnce(this::stopIndexerAndFeeder);
    }

    public Command runShooterCommand() {
        return this.runOnce(this::runShooter);
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
        
        // NEW: Show the raw percentage power (0.0 to 1.0) going to the motors
        SmartDashboard.putNumber("Shooter Raw Power", m_NEOshooter.getAppliedOutput());
        
        SmartDashboard.putBoolean("Shooter Is At Speed", isAtSpeed());    
        
    }
}