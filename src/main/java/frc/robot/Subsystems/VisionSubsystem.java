package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        
        // Setup Video Streams for the Driver Dashboard
        CameraServer.startAutomaticCapture();
        CameraServer.addAxisCamera("LeftCamera_Stream", "http://limelight-left.local:5800/stream.mjpg");
        CameraServer.addAxisCamera("RightCamera_Stream", "http://limelight-right.local:5800/stream.mjpg");
    }
    
    // --- ODOMETRY UPDATES ARE NOW SAFELY HANDLED IN DRIVESUBSYSTEM ---

    public double getTx() {
        return LimelightHelpers.getTX("LeftCamera");
    }

    public double getTy() {
        return LimelightHelpers.getTY("LeftCamera"); 
    }

    public boolean hasTarget() {
        // Simply return true if the camera sees any valid target
        return LimelightHelpers.getTV("LeftCamera");
    }
    
    public double getDistanceToTarget() {
        return getTy();
    }
    
    @Override
    public void periodic() {
        // Only publish targeting data for the Aim-Bot, no odometry math here!
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Error", getTx());
        SmartDashboard.putNumber("Limelight Y Error", getTy());
    }
}