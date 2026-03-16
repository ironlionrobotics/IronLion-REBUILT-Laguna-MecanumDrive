package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final Timer m_timer = new Timer();
    private final double m_duration;

    public RunIntakeCommand(IntakeSubsystem intake, double duration) {
        m_intakeSubsystem = intake;
        m_duration = duration;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }
}

