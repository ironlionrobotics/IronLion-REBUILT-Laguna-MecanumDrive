package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final MatchStateSubsystem m_matchState;
    private final Timer m_blinkTimer = new Timer();
    
    // Estado de alineación de PathPlanner / Visión
    private boolean m_isAlignedToHub = false;

    public LEDSubsystem(MatchStateSubsystem matchState) {
        this.m_matchState = matchState;
        
        m_led = new AddressableLED(LEDConstants.kPWMPort);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLedLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        m_blinkTimer.start();
    }

    public void setAlignedToHub(boolean aligned) {
        this.m_isAlignedToHub = aligned;
    }

    @Override
    public void periodic() {
        if (m_isAlignedToHub) {
            // Efecto de parpadeo BLANCO indicando que el conductor debe disparar!
            if ((int)(m_blinkTimer.get() * 5) % 2 == 0) {
                setAllColor(255, 255, 255); // Blanco
            } else {
                setAllColor(0, 0, 0); // Apagado
            }
        } else if (m_matchState.isHubActive()) {
            // HUB está ACTIVO: Color VERDE
            // Si le quedan menos de 5 segundos, empieza a parpadear
            if (m_matchState.getTimeRemainingInShift() <= 5.0) {
                 if ((int)(m_blinkTimer.get() * 10) % 2 == 0) {
                    setAllColor(0, 255, 0);
                 } else {
                    setAllColor(0, 0, 0);
                 }
            } else {
                setAllColor(0, 255, 0); // Verde sólido
            }
        } else {
            // HUB INACTIVO: Color ROJO
            setAllColor(255, 0, 0);
        }

        m_led.setData(m_ledBuffer);
    }

    private void setAllColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }
}
