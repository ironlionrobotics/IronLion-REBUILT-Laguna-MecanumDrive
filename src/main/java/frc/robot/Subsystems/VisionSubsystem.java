package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Inteligencia Optica (Visión)
 * Utiliza cámaras inteligentes modulares (Limelights) para identificar targets AprilTag (QR Códigos del Campo).
 * Inyecta estos rastros como datos para corregir el Sistema de Navegación del Chassis y permite "AimBots" automáticos.
 */
public class VisionSubsystem extends SubsystemBase {

    // Tablas de red inalámbricas que la Limelight provee con toda la telemetría viva para que Java la abstraiga
    private final NetworkTable m_frontCameraTable;
    private final NetworkTable m_backCameraTable;
    
    // Necesitamos comunicación directa con Drive. Si la cámara sabe que nos movimos a un X/Y absoluto por ver un tag, 
    // debe decírselo al Drive para que recalibre sus motores (Sensor Fusion).
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;
        // Levantando servidores UDP ocultos en las "NetworkTables" para escuchar a los IPs de la cámara interna.
        m_frontCameraTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kFrontCameraName);
        m_backCameraTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kBackCameraName);
    }

    /**
     * Analiza el streaming de datos vivo de la tabla. Si hay un AprilTag a la vista en 3D (MegaTag Mode), 
     * lo desencripta y le dice al Drive "Aquí es donde realmente estás en el mundo real respecto a tu punto de inicio".
     */
    private void processLimelight(NetworkTable table, String name) {
        // "tv" (Target Valid): Regresa 1.0 si la cámara encontró a la IA algorítmica y cuadra con los targets que buscamos. 
        double hasTarget = table.getEntry("tv").getDouble(0);
        
        if (hasTarget == 1.0) {
            // "botpose_wpiblue": Es una característica monstruosa (MegaTag Pose Estimator) que saca la Coordenada Mundial del robot 
            // asumiendo que el campo mide (X) metros y que el origen (0,0) está siempre en la esquina inferior izquierda de la alianza azul.
            double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            
            if (botpose.length == 6) {
                // Formato de botpose (Arraigo) devuelto: [x_campo, y_campo, z_elevación, roll, pitch, yaw(Cuerpos giremos de Z)]
                double x = botpose[0];
                double y = botpose[1];
                double yaw = botpose[5]; // Cuánto está volteado para chocar o correr

                Pose2d visionPose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(yaw));
                
                // Cálculo de latencia en milisegundos preciso para Kalman Filter (Filtros Predictivos de Odometría):
                // A qué exacto ms la cámara vió la imágen vs cuando nos lo envió por Ethernet.
                double tl = table.getEntry("tl").getDouble(0); // target latency computing
                double cl = table.getEntry("cl").getDouble(0); // ethernet communication latency capture
                // Obtenemos el timestamp del procesador restándole lo que tardó el "lag".
                double timestamp = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
                
                // Le pasamos todo finalmente masticado al Estimador avanzado (ADAS) de Mecanum en DriveSubsystem.
                m_driveSubsystem.addVisionMeasurement(visionPose, timestamp);

                // Mandamos datos simplificados para humanos y diagramas espaciales al UI de la Driver Station Web
                SmartDashboard.putNumber("Vision/" + name + "_X", x);
                SmartDashboard.putNumber("Vision/" + name + "_Y", y);
            }
        }
    }

    /** "Target X" -> Cuántos grados de error hacia Izquierda(-) o Derecha(+) hay para apuntarle directo al HUB desde la cámara */
    public double getTx() {
        return m_frontCameraTable.getEntry("tx").getDouble(0.0);
    }

    /** "Target Y" -> Cuántos grados de declive hacia Abajo(-) o Arriba(+) está visualmente el HUB respecto al centro de nuestra cámara lente */
    public double getTy() {
        return m_frontCameraTable.getEntry("ty").getDouble(0.0);
    }

    /** Chequear directo y rápido el pulso si vemos un tag verde. Útil para hacer trigger Booleano rápido de LEDS */
    public boolean hasTarget() {
        return m_frontCameraTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Abstracción sobre la altura y ty, devueve distancia a usar por parte de nuestra Torreta virtual en fase 5.
     */
    public double getDistanceToTarget() {
        // Matemáticas rigurosas Pitagóricas (Opcionales que requieren calibración métrica pre-Cancha): 
        // distance = (HubHeightMeters - CameraHeightMeters) / tan(CameraMountAngleRadians + TargetYDegrees)
        
        // Versión funcional actual "Cruda y Absoluta": Devolvemos Ty y obligamos en el Shooter a mapear 
        // de Ty grados directamente contra RPMs ganadoras ensayadas experimentalmente (Muchísimo mas estable si tiembla).
        return getTy(); 
    }

    /** El latido constante de procesamiento. 50 fotogramas por segundo mandados a Odometría Global */
    @Override
    public void periodic() {
        processLimelight(m_frontCameraTable, VisionConstants.kFrontCameraName);
        // Si hay una Limelight atràs (Rear), ayudaría inmensamente cuando vayamos en reversa ciega, procesala tb!
        processLimelight(m_backCameraTable, VisionConstants.kBackCameraName);
    }
}
