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

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable m_frontCameraTable;
    private final NetworkTable m_backCameraTable;
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;
        // Asignamos las tablas de NetworkTables donde las Limelights publican los datos
        m_frontCameraTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kFrontCameraName);
        m_backCameraTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kBackCameraName);
    }

    /**
     * Lee la odometría de la cámara (MegaTag o AprilTags estándar) y se la envía al DriveSubsystem
     * para que corrija el MecanumDrivePoseEstimator.
     */
    private void processLimelight(NetworkTable table, String name) {
        // tv = 1.0 si la cámara ve un target válido (AprilTag en este caso).
        double hasTarget = table.getEntry("tv").getDouble(0);
        
        if (hasTarget == 1.0) {
            // botpose_wpiblue nos da la pose orientada al campo asumiendo alianza azul (Origin Bottom-Right)
            // Para la alianza roja, WPILib puede auto-reflejarla con AutoBuilder o lo hace el mismo Limelight
            double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            
            if (botpose.length == 6) {
                // Formato de botpose: [x, y, z, roll, pitch, yaw]
                double x = botpose[0];
                double y = botpose[1];
                double yaw = botpose[5];

                Pose2d visionPose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(yaw));
                
                // Cálculo preciso del tiempo para MegaTag / Kalman Filter
                double tl = table.getEntry("tl").getDouble(0); // target pipeline latency
                double cl = table.getEntry("cl").getDouble(0); // capture latency
                double timestamp = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
                
                // Proveer el dato corregido al Chassis Pose Estimator
                m_driveSubsystem.addVisionMeasurement(visionPose, timestamp);

                // Imprimir para diagnóstico
                SmartDashboard.putNumber("Vision/" + name + "_X", x);
                SmartDashboard.putNumber("Vision/" + name + "_Y", y);
            }
        }
    }

    public double getTx() {
        return m_frontCameraTable.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return m_frontCameraTable.getEntry("ty").getDouble(0.0);
    }

    public boolean hasTarget() {
        return m_frontCameraTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Calcula la distancia estimada basándose en el ángulo ty de la Limelight.
     * Requiere que kCameraHeight, kTargetHeight y kCameraPitch estén calibrados en Constants.
     */
    public double getDistanceToTarget() {
        // En un escenario real, usarías:
        // double angleToGoalDegrees = VisionConstants.kCameraPitch + getTy();
        // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        // return (VisionConstants.kTargetHeight - VisionConstants.kCameraHeight) / Math.tan(angleToGoalRadians);
        
        // Por ahora devolvemos un cálculo simplificado o ty crudo para la tabla
        return getTy(); 
    }

    @Override
    public void periodic() {
        processLimelight(m_frontCameraTable, VisionConstants.kFrontCameraName);
        processLimelight(m_backCameraTable, VisionConstants.kBackCameraName);
    }
}
