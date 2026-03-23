package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        CameraServer.startAutomaticCapture();
    }
    
    public void processLimelight(String cameraName) {
        if (LimelightHelpers.getTV(cameraName)) {
            LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

            if (poseEstimate.tagCount > 0) {
                m_driveSubsystem.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                SmartDashboard.putNumber(cameraName + "_X", poseEstimate.pose.getX());
                SmartDashboard.putNumber(cameraName + "_Y", poseEstimate.pose.getY());
            }
        }
    }
    public String getActiveCamera() {       
        if (LimelightHelpers.getTV("RightCamera")) {
            return "RightCamera";
        } else if (LimelightHelpers.getTV("LeftCamera")) {
            return "LeftCamera";
        }
        return "LeftCamera"; // Default fallback if neither see anything
    }
    public double getTx() {
        return LimelightHelpers.getTX("LeftCamera");
    }

    public double getTy() {
        return LimelightHelpers.getTY("LeftCamera"); // Fixed to TY
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV("LeftCamera");
    }
    
    public double getDistanceToTarget() {
        return getTy();
    }
    
    @Override
    public void periodic() {
        processLimelight("LeftCamera"); 
        processLimelight("RightCamera"); 

        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Error", getTx());
        SmartDashboard.putNumber("Limelight Y Error", getTy());
    }
}