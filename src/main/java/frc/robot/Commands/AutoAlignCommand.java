package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class AutoAlignCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final PIDController m_controller;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        m_driveSubsystem = drive;
        m_visionSubsystem = vision;
        m_controller = new PIDController(VisionConstants.kAlignP, VisionConstants.kAlignI, VisionConstants.kAlignD);
        m_controller.setTolerance(VisionConstants.kAlignTolerance);
        
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        if (m_visionSubsystem.hasTarget()) {
            double rotationSpeed = m_controller.calculate(m_visionSubsystem.getTx(), 0);
            // Solo rotamos, permitimos que el conductor siga controlando X e Y si es necesario (modo asistido)
            // O podemos dejarlo fijo en 0,0 para alineación pura.
            m_driveSubsystem.drive(0, 0, rotationSpeed, false);
        } else {
            m_driveSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stop();
    }

    public boolean isAligned() {
        return m_visionSubsystem.hasTarget() && m_controller.atSetpoint();
    }
}
