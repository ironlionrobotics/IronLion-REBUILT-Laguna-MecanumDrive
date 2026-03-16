package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Sistema Chronos: Realiza un seguimiento del tiempo del partido vía FMS
 * para calcular los "Shifts" de 25 segundos y el estado de los HUBs (Reglas de REBUILT 2026).
 */
public class MatchStateSubsystem extends SubsystemBase {

    // Constante del juego REBUILT
    private static final double SHIFT_DURATION = 25.0; // Segundos
    private static final double TELEOP_START_TIME = 135.0; // El match.time() de FMS cuenta hacia atrás

    private boolean m_isHubActive = false;
    private double m_timeRemainingInShift = 0.0;

    public MatchStateSubsystem() {}

    @Override
    public void periodic() {
        if (DriverStation.isTeleopEnabled()) {
            // FMS Match Time va de 135 a 0 durante Teleop
            double matchTime = DriverStation.getMatchTime();
            
            // Calculamos el tiempo transcurrido desde que inició el teleop
            double timeElapsed = TELEOP_START_TIME - matchTime;

            if (timeElapsed >= 0 && matchTime >= 0) {
                // Módulo 50 porque el ciclo completo es 50s (25 activo, 25 inactivo)
                // Asumiendo, por diseño del juego REBUILT, que el HUB empieza inactivo o activo.
                // Ajustar el offset según las reglas exactas si empiezan inactivos.
                // Aquí asumimos que los primeros 25s son ACTIVOS, luego 25s INACTIVOS
                double shiftCycle = timeElapsed % (SHIFT_DURATION * 2);

                if (shiftCycle < SHIFT_DURATION) {
                    m_isHubActive = true;
                    m_timeRemainingInShift = SHIFT_DURATION - shiftCycle;
                } else {
                    m_isHubActive = false;
                    m_timeRemainingInShift = (SHIFT_DURATION * 2) - shiftCycle;
                }
            } else {
                m_isHubActive = false;
                m_timeRemainingInShift = 0;
            }
        } else {
            // Fuera de teleop, el HUB puede estar inactivo.
            m_isHubActive = false;
            m_timeRemainingInShift = 0;
        }
    }

    /**
     * @return true si el HUB de tu alianza está iluminado y listo para recibir FUEL.
     */
    public boolean isHubActive() {
        return m_isHubActive;
    }

    /**
     * @return Segundos restantes antes de que el HUB actual cambie de estado.
     */
    public double getTimeRemainingInShift() {
        return m_timeRemainingInShift;
    }
}
