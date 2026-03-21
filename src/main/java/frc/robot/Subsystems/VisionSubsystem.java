package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
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

    public double getTx() {
        return LimelightHelpers.getTX("CAM1");
    }

    public double getTy() {
        return LimelightHelpers.getTY("CAM1"); // Fixed to TY
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV("CAM1");
    }
    
    public double getDistanceToTarget() {
        return getTy();
    }
    
    @Override
    public void periodic() {
        processLimelight("CAM1"); // Ensure vision is processed
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Error", getTx());
        SmartDashboard.putNumber("Limelight Y Error", getTy());
    }
}