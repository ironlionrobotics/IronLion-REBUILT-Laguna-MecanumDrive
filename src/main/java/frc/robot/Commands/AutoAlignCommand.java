package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import java.util.function.DoubleSupplier;

public class AutoAlignCommand extends Command {
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;

    private final PIDController m_rotationPID;
    private final PIDController m_distancePID;

    private final DoubleSupplier m_avanzarSupplier;
    private final DoubleSupplier m_lateralSupplier;

    public AutoAlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier avanzarSupplier, DoubleSupplier lateralSupplier) {   
        m_drive = driveSubsystem;
        m_vision = visionSubsystem;
        m_lateralSupplier = lateralSupplier;
        m_avanzarSupplier = avanzarSupplier;

        m_rotationPID = new PIDController(
            VisionConstants.kAlignP,
            VisionConstants.kAlignI,
            VisionConstants.kAlignD
        );
        m_rotationPID.setSetpoint(0.0); // We always want tx to be 0
        m_rotationPID.setTolerance(VisionConstants.kAlignTolerance);

        m_distancePID = new PIDController(
            VisionConstants.kDistanceP,
            VisionConstants.kDistanceI,
            VisionConstants.kDistanceD
        );
        m_distancePID.setSetpoint(VisionConstants.kTargetTy); 
        m_distancePID.setTolerance(VisionConstants.kDistanceTolerance);

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double deadZone = 0.2; // Match the new standard deadband
        
        // 1. Let the driver control forward/backward and side-to-side ALWAYS
        double lateral = -MathUtil.applyDeadband(m_lateralSupplier.getAsDouble(), deadZone) * 3.0;
        
        // 2. Default rotation to 0
        double rotate = 0.0;
        double avanzar = 0.0;
        // 3. ONLY override rotation if Limelight sees the target
        if (m_vision.hasTarget()) {
           rotate = m_rotationPID.calculate(m_vision.getTx());
           
        }
        
        // 4. Build the chassis speeds and send them to the drivetrain
        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
            edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                avanzar, 
                lateral, 
                rotate, 
                m_drive.getHeading()
            );          
        m_drive.driveRobotRelative(velocidades);
    }

    public boolean isAligned() {
        return m_vision.hasTarget() && m_rotationPID.atSetpoint() && m_distancePID.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drive.driveRobotRelative(new ChassisSpeeds(0,0,0));
    }
}