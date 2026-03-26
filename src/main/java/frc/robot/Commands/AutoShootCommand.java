package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class AutoShootCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final VisionSubsystem m_vision;
    private final Timer m_feedTimer = new Timer();
    private boolean m_isFeeding = false;

    public AutoShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        m_shooter = shooter;
        m_vision = vision;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_feedTimer.reset();
        m_feedTimer.start();
        m_isFeeding = false;
    }

    @Override
        public void execute() {
            if (m_vision.hasTarget()) {
                m_shooter.runShooter(); // Sends raw 1.0 Tunable speed
                
                // Wait 0.5s for inertia before feeding
                if (m_feedTimer.hasElapsed(0.5)) {
                    m_shooter.runIndexerAndFeeder();
                    m_isFeeding = true; 
                }
            } else {
                m_shooter.stopShooter();
                m_shooter.stopIndexerAndFeeder();
                m_feedTimer.reset();
            }
        }

    @Override
    public boolean isFinished() {
        return m_isFeeding && m_feedTimer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
        m_shooter.stopIndexerAndFeeder();
    }
}