package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Sistema Chronos: Realiza un seguimiento inteligente del tiempo del partido basado en el 
 * reloj oficial del campo provisto por el Field Management System (FMS).
 * Calcula si la estructura HUB del campo está en estado Activo y cuánto falta para cerrarse y resetear.
 */
public class MatchStateSubsystem extends SubsystemBase {

    // Constante matemática dictada por FRC REBUILT 2026. 
    // Los HUBs permanecen encendidos 25 segundos y se apagan los siguientes 25.
    private static final double SHIFT_DURATION = 25.0; // Segundos
    // El partido Teleoperado siempre inicia con 135 Segundos de vida y el FMS tira la regresiva.
    private static final double TELEOP_START_TIME = 135.0; 

    // Banderas internas que actúan de memoria local.
    private boolean m_isHubActive = false;
    private double m_timeRemainingInShift = 0.0;

    public MatchStateSubsystem() {}

    /**
     * Periodic() es el pulso cardíaco del subsistema. Ejecutará esta lógica cada 20ms en el fondo de todo lo que el robot haga.
     */
    @Override
    public void periodic() {
        // Solo calcular datos en Teleop donde apliquen las dinámicas Shift (Los hubs en Autónomo actúan diferente y fijos).
        if (DriverStation.isTeleopEnabled()) {
            
            // Revisa el tiempo restante. Ejemplo: Inicia en 135.2 y caerá.
            double matchTime = DriverStation.getMatchTime();
            
            // Transformamos el dato FMS regresivo a tiempo "Transcurrido". (Contar hacia arriba desde 0)
            double timeElapsed = TELEOP_START_TIME - matchTime;

            if (timeElapsed >= 0 && matchTime >= 0) {
                // Matemática de ciclo "Módulo" %: 
                // Explicación: 50 segundos es un lapso gigantecimiento (25 ON y 25 OFF).
                // Con %=50, el timer transcurrido ej: los 74 segundos, darán residuo "24".
                double shiftCycle = timeElapsed % (SHIFT_DURATION * 2);

                if (shiftCycle < SHIFT_DURATION) { // Menor a 25s (El residuo cuadra en la zona activa)
                    m_isHubActive = true;
                    // Faltante = Límite (25) menos el pulso residual transitable.
                    m_timeRemainingInShift = SHIFT_DURATION - shiftCycle;
                } else { // Pasó de 25s por ende está apagado el ciclo.
                    m_isHubActive = false;
                    m_timeRemainingInShift = (SHIFT_DURATION * 2) - shiftCycle;
                }
            } else {
                m_isHubActive = false;
                m_timeRemainingInShift = 0;
            }
        } else {
            // Fuera de teleoperado, no bloquearemos por regla. 
            m_isHubActive = false;
            m_timeRemainingInShift = 0;
        }
    }

    /**
     * Informa al exterior (RobotContainer o Triggers) si se debe obedecer el flujo del Hub ahora.
     * @return true si el HUB de tu alianza está iluminado según reglas y acepta FUEL.
     */
    public boolean isHubActive() {
        return m_isHubActive;
    }

    /**
     * Devuelve el tiempo restante en formato decimal exacto.
     * Ejemplo para desarrollador UI/UX: Modificar el color del timer de verde a rojo palpitante cuando llegue a 5 seg.
     * @return Segundos restantes antes de que el HUB cambie su vida útil.
     */
    public double getTimeRemainingInShift() {
        return m_timeRemainingInShift;
    }
}
