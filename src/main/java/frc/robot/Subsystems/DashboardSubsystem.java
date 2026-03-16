package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.MatchStateSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Constants.TunableConstants;

/**
 * Este subsistema actúa analógicamente como el "Reportero Local" del robot. 
 * Se dedica exclusivamente a leer los signos vitales profundos de todos los demás hardware conectados
 * y publicarlos crudos en la nube robotica via NetworkTables4 (NT4)
 * de donde nuestra Web App (SpaceX) en Vite/React consumirá los datos a latencia nula.
 * OJO: Aquí NO se mueven motores, solo se emiten boletines.
 */
public class DashboardSubsystem extends SubsystemBase {
    
    // Referencias a las venas de donde se extraerá el plasma (información)
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

    /** Ciclo de reporte de streaming ininterrumpido (Tasa 20ms refresco) */
    @Override
    public void periodic() {
        // --- ACTUALIZAR VALORES TUNABLES DESDE SMARTDASHBOARD ---
        // Cada 20ms lee los valores que el equipo puede haber editado desde la laptop del pit.
        TunableConstants.updateFromDashboard();
        // 1. CHRONOS (Lógica Oficial REBUILT)
        // =========================================================
        // Crea KeyValues que la Web leerá con el prefijo "WebDash/" 
        SmartDashboard.putBoolean("WebDash/HubActive", m_matchState.isHubActive());
        SmartDashboard.putNumber("WebDash/ShiftTimeRemaining", m_matchState.getTimeRemainingInShift());

        // =========================================================
        // 2. CHECKLIST DE SALUD (Lista de Vuelo del Mecánico)
        // Analíticas primordiales para prevenir tragedias previo al pito de inicio
        // =========================================================
        double batteryVoltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("WebDash/BatteryVoltage", batteryVoltage);
        // Si la batería baja de 12V se pondrá a vibrar alarmas en Front-End.
        SmartDashboard.putBoolean("WebDash/BatteryOK", batteryVoltage > 12.0);

        // Si se nos desenchufa o muere the Pigeon2, no sabremos manejar derecho núnca
        boolean isGyroReady = m_drive.getGyro().isConnected();
        SmartDashboard.putBoolean("WebDash/GyroOK", isGyroReady);

        // Revisa que una Limelight llamada `front` esté publicando tablas de Visión (`tv` means target view status).
        boolean isFrontCamConnected = NetworkTableInstance.getDefault().getTable("limelight-front").containsEntry("tv");
        SmartDashboard.putBoolean("WebDash/CameraOK", isFrontCamConnected);

        // =========================================================
        // 3. ADAS HUD (Mini-Mapa Holográfico Virtual)
        // =========================================================
        // Exporta X,Y,Angulo del GPS al react app para trazar visualmente el dibujo del robot
        SmartDashboard.putNumber("WebDash/RobotX", m_drive.getPose().getX());
        SmartDashboard.putNumber("WebDash/RobotY", m_drive.getPose().getY());
        SmartDashboard.putNumber("WebDash/RobotRotation", m_drive.getPose().getRotation().getDegrees());

        // Master Node Alert (Si esto cae, manda un destello ambar o alerta web).
        boolean allSystemsGo = (batteryVoltage > 12.0) && isGyroReady && isFrontCamConnected;
        SmartDashboard.putBoolean("WebDash/AllSystemsGo", allSystemsGo);

        // =========================================================
        // 4. TELEMETRÍA DEL CANNON ADAPTATIVO
        // =========================================================
        // Datos vivos exactos en RPM (Revoluciones) medidas directo sobre la flecha metálica del lanzador principal 
        SmartDashboard.putNumber("WebDash/ShooterRPM", m_shooter.getShooter().getEncoder().getVelocity());
        SmartDashboard.putNumber("WebDash/VisionTx", m_vision.getTx());
        // Indicador binario si en la vida real consideramos esa mira digna de "Anotación Limpia Segura" (< 1 grados de chueco)
        SmartDashboard.putBoolean("WebDash/Aligned", Math.abs(m_vision.getTx()) < 1.0);
    }
}
