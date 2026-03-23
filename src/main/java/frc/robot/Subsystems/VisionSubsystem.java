package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        CameraServer.startAutomaticCapture();
        CameraServer.addAxisCamera("LeftCamera_Stream", "http://limelight-left.local:5800/stream.mjpg");
        CameraServer.addAxisCamera("RightCamera_Stream", "http://limelight-right.local:5800/stream.mjpg");
        m_driveSubsystem = driveSubsystem;
    }
    
    public void processLimelight(String cameraName) {
        if (LimelightHelpers.getTV(cameraName)) {
            LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

            if (poseEstimate != null && poseEstimate.tagCount > 0) {
                m_driveSubsystem.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                SmartDashboard.putNumber(cameraName + "_X", poseEstimate.pose.getX());
                SmartDashboard.putNumber(cameraName + "_Y", poseEstimate.pose.getY());
            }
        }
    }


    public double getTx() {
        return LimelightHelpers.getTX("LeftCamera");
    }

    public double getTy() {
        return LimelightHelpers.getTY("LeftCamera"); 
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV("LeftCamera");
    }
    
    public double getDistanceToTarget() {
        return getTy();
    }
    
    @Override
    public void periodic() {
        //processLimelight("RightCamera"); 
        
        LimelightHelpers.SetRobotOrientation("LeftCamera", 
        m_driveSubsystem.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        
        processLimelight("LeftCamera"); 
        processLimelight("RightCamera"); 
        
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Error", getTx());
        SmartDashboard.putNumber("Limelight Y Error", getTy());
    }
}