package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.DriveSubsystem;

public class FollowTagCommand extends Command {
    private final DriveSubsystem m_drive;
    private final PIDController m_distanceController;
    private final PIDController m_rotationController;

    public FollowTagCommand(DriveSubsystem drive) {
        m_drive = drive;
        m_distanceController = new PIDController(VisionConstants.kDistanceP, 0, 0);
        m_rotationController = new PIDController(VisionConstants.kAlignP, 0, VisionConstants.kAlignD);
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        boolean leftSeesTarget = LimelightHelpers.getTV("limelight-left");
        boolean rightSeesTarget = LimelightHelpers.getTV("limelight-right");
        
        double tx = 0.0;
        double ty = 0.0;
        boolean hasTarget = false;

        if (rightSeesTarget) {
            tx = LimelightHelpers.getTX("limelight-right");
            ty = LimelightHelpers.getTY("limelight-right");
            
            hasTarget = true;
        }
        // if (leftSeesTarget && rightSeesTarget) {
        //     tx = (LimelightHelpers.getTX("limelight-left") + LimelightHelpers.getTX("limelight-right")) / 2.0;
        //     ty = (LimelightHelpers.getTY("limelight-left") + LimelightHelpers.getTY("limelight-right")) / 2.0;
        //     hasTarget = true;
        // } else if (leftSeesTarget) {
        //     tx = LimelightHelpers.getTX("limelight-left");
        //     ty = LimelightHelpers.getTY("limelight-left");
        //     hasTarget = true;
        // } else if (rightSeesTarget) {
        //     tx = LimelightHelpers.getTX("limelight-right");
        //     ty = LimelightHelpers.getTY("limelight-right");
        //     hasTarget = true;
        // }

        if (!hasTarget) {
            m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            return;
        }

        double forwardSpeed = m_distanceController.calculate(ty, VisionConstants.kTargetTy);
        double rotationSpeed = -m_rotationController.calculate(tx, 0.0);
        
        forwardSpeed = MathUtil.clamp(forwardSpeed, -2.0, 3.0);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -2.0, 3.0);

        
        m_drive.driveRobotRelative(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }
}   