package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.MatchStateSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

/**
 * Recopila todos los signos vitales del robot para transmitirlos 
 * al Dashboard Web Custom (Nivel 2) a través de NetworkTables 4 (NT4).
 */
public class DashboardSubsystem extends SubsystemBase {
    
    private final DriveSubsystem m_drive;
    private final MatchStateSubsystem m_matchState;
    private final VisionSubsystem m_vision;
    private final ShooterSubsystem m_shooter;

    public DashboardSubsystem(DriveSubsystem drive, MatchStateSubsystem matchState, VisionSubsystem vision, ShooterSubsystem shooter) {
        this.m_drive = drive;
        this.m_matchState = matchState;
        this.m_vision = vision;
        this.m_shooter = shooter;
    }

    @Override
    public void periodic() {
        // 1. CHRONOS: Estado del Hub y Reloj
        SmartDashboard.putBoolean("WebDash/HubActive", m_matchState.isHubActive());
        SmartDashboard.putNumber("WebDash/ShiftTimeRemaining", m_matchState.getTimeRemainingInShift());

        // 2. PRE-MATCH CHECKLIST: Salud del Sistema
        double batteryVoltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("WebDash/BatteryVoltage", batteryVoltage);
        SmartDashboard.putBoolean("WebDash/BatteryOK", batteryVoltage > 12.0);

        // Giroscopio Salud
        boolean isGyroReady = m_drive.getGyro().isConnected();
        SmartDashboard.putBoolean("WebDash/GyroOK", isGyroReady);

        // Conexión de Cámara (Ej. Limelight Frontal transmitiendo NT)
        boolean isFrontCamConnected = NetworkTableInstance.getDefault().getTable("limelight-front").containsEntry("tv");
        SmartDashboard.putBoolean("WebDash/CameraOK", isFrontCamConnected);

        // 3. ADAS HUD: Coordenadas de Odometría para el Mini-Mapa 3D/2D
        SmartDashboard.putNumber("WebDash/RobotX", m_drive.getPose().getX());
        SmartDashboard.putNumber("WebDash/RobotY", m_drive.getPose().getY());
        SmartDashboard.putNumber("WebDash/RobotRotation", m_drive.getPose().getRotation().getDegrees());

        // Estatus general pre-match para el "GREEN LIGHT" del Dashboard
        boolean allSystemsGo = (batteryVoltage > 12.0) && isGyroReady && isFrontCamConnected;
        SmartDashboard.putBoolean("WebDash/AllSystemsGo", allSystemsGo);

        // Phase 5: Telemetría de Disparo Adaptativo
        SmartDashboard.putNumber("WebDash/ShooterRPM", m_shooter.getShooter().getEncoder().getVelocity());
        SmartDashboard.putNumber("WebDash/VisionTx", m_vision.getTx());
        SmartDashboard.putBoolean("WebDash/Aligned", Math.abs(m_vision.getTx()) < 1.0);
    }
}
