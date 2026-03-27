package frc.robot.Subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
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

    private void updateVisionOdometry() {
        String[] cameraNames = {"LeftCamera", "RightCamera"};

        for (String camera : cameraNames) {
            LimelightHelpers.SetRobotOrientation(
                camera,
                m_driveSubsystem.getHeading().getDegrees(),
                0, 0, 0, 0, 0
            );

            LimelightHelpers.PoseEstimate limelightMeasurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);

            if (limelightMeasurement == null || limelightMeasurement.tagCount == 0) {
                continue;
            }

            // Reject highly ambiguous single-tag data
            if (limelightMeasurement.tagCount == 1 && limelightMeasurement.rawFiducials.length > 0) {
                if (limelightMeasurement.rawFiducials[0].ambiguity > 0.7) {
                    continue;
                }
            }

            double xyStds;
            if (limelightMeasurement.tagCount >= 2) {
                xyStds = 0.1; // High trust: multiple tags cancel out ambiguity
            } else if (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 2.0) {
                xyStds = 0.3; // Medium trust: 1 tag, but close
            } else {
                xyStds = 1.0; // Low trust: 1 tag, far away
            }

            double degStds = 9999999; // Never trust Limelight rotation, trust the Pigeon gyro

            m_driveSubsystem.setVisiox  xnMeasurementStdDevs(xyStds, degStds);
            m_driveSubsystem.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds
            );
        }
    }

    @Override
    public void periodic() {
        updateVisionOdometry();

        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight X Error", getTx());
        SmartDashboard.putNumber("Limelight Y Error", getTy());
    }
}