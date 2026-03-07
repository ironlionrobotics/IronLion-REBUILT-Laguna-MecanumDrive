package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private double m_duration;
    private long m_startTime;

    public RunIntakeCommand(IntakeSubsystem intake, double duration) {
        m_intakeSubsystem = intake;
        m_duration = duration;
        m_startTime = System.currentTimeMillis();
        addRequirements(m_intakeSubsystem); //declarar requerimientos para evitar conflictos con otros comandos que usen el mismo subsistema
    }

    public void initialize() {
        m_intakeSubsystem.runIntake(); // ohhhh
        m_startTime = System.currentTimeMillis(); // Reiniciar el tiempo de inicio al iniciar el comando
    }
    public void end() {
        m_intakeSubsystem.stopIntake(); // Detener el intake al finalizar el comando
    }

}

