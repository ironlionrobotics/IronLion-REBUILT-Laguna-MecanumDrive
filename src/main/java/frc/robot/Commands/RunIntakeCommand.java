package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

/**
 * Comando manual sub-independiente fabricado especialmente a la medida para la API gráfica (PathPlanner) 
 * controlando y accionando la barredora mecánica de absorción.
 * 
 * ¿En qué difiere frente a lo manual?
 * En lugar de correr al infinito y más allá si una IA virtual de trayectorias le oprime,
 * se frena así mismo en autocontrol pasandose como antídoto un valoe 'duration' limitador nativo a su cronómetro en instanciación (Al arrancar base).
 * 
 * En PathPlanner los pilotos armarán combos con él: 
 * Ej: "Mi trayecto es recoger eso azul... Oh! Prenderé MIENTRAS camino el RunIntakeCommand exigiendo límite rancio forzado de EXACTOS 5.2 (segs) solo para raspar".
 */
public class RunIntakeCommand extends Command {
    
    private final IntakeSubsystem m_intakeSubsystem;
    
    // Reloj digital robótico (Hardware Ticked) para saber la duración viva actual 
    private final Timer m_timer = new Timer();
    
    // Tiempo Límite Mortal Exigido
    private final double m_duration;

    /**
     * @param intake Requisición inyectada obligatoria de componentes abstractos del bloque Recolector.
     * @param duration Plazo dictatorial cruel de vida de devocarión de piezas a tolerar (En Segundos Decimales).
     */
    public RunIntakeCommand(IntakeSubsystem intake, double duration) {
        m_intakeSubsystem = intake;
        m_duration = duration;
        
        // Evade la duplicación destructiva lógica. (Que haya dos comandos robando energía al SubSystem).
        addRequirements(m_intakeSubsystem);
    }

    /** 
     * Inicio oficial pre-arranque. 
     */
    @Override
    public void initialize() {
        m_timer.restart(); // Reinicia métricas obsoletas para arrancar en el Limbo a "0" Segs
        m_intakeSubsystem.runIntake(); // Encendido ruidoso y violento del engranaje verde recogedor!
    }

    /** 
     * Condena a Muerte limpia terminante del bloque. Ocurre si "isFinished()" lanza 'TRUE'.
     */
    @Override
    public void end(boolean interrupted) {
        m_timer.stop(); // Cortar Relojes pasivos.
        m_intakeSubsystem.stopIntake(); // Seca los VDC en 0 para evadir gastadera del RoboRIO e inercia inútil rodada del FMS.
    }

    /** 
     * Péndulo de la Muerte. La validación robótica maestra que manda a limpiar este Command al basurero virtual una vez complido la meta.
     * True indica terminar.
     */
    @Override
    public boolean isFinished() {
        // Validación del Crono Atómico... ¿Ya me he extendido temporalmente la cuota máxima numérica Seconds impuesta desde parámetros de instanciación? 
        return m_timer.hasElapsed(m_duration);
    }
}
