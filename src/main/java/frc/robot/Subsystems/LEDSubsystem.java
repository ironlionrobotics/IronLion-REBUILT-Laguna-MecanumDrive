package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Controla la retroalimentación visual del robot a través de tiras LED NeoPixel.
 * Ayuda a los conductores a saber si el robot está preparado para disparar sin necesidad
 * de mirar la pantalla de la computadora en medio de la cancha.
 */
public class LEDSubsystem extends SubsystemBase {
    // Hardware nativo de WPILib para manejar luces encadenadas RGB digitales
    private final AddressableLED m_led;
    // Buffer de memoria donde "dibujamos" los colores antes de mandarlos al procesador LED
    private final AddressableLEDBuffer m_ledBuffer;
    
    // El LEDSubsystem necesita saber en qué estado temporal está el match (para brillar Verde o Rojo según REBUILT Shift)
    private final MatchStateSubsystem m_matchState;
    
    // Temporizador utilizado para crear los efectos visuales de "parpadeo tipo estribo"
    private final Timer m_blinkTimer = new Timer();
    
    // Bandera(Memoria) que indica si actualmente la mira de nuestra cámara está en el blanco perfecto
    private boolean m_isAlignedToHub = false;

    public LEDSubsystem(MatchStateSubsystem matchState) {
        this.m_matchState = matchState;
        
        m_led = new AddressableLED(LEDConstants.kPWMPort);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLedLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        
        m_led.start(); // Encender la transmisión DMA desde el RoboRIO

        m_blinkTimer.start(); // Arrancar el cronómetro maestro de luces al infinito
    }

    /** Permite que otros subsistemas y triggers externos le avisen a este subsistemsi lograron alinear al Hub */
    public void setAlignedToHub(boolean aligned) {
        this.m_isAlignedToHub = aligned;
    }

    /** 
     * Función obligatoria. Aquí es donde se pintan los fotogramas dinámicos 50 veces por seg. 
     */
    @Override
    public void periodic() {
        if (m_isAlignedToHub) {
            // EFECTO 1: "DISPARA YA!"
            // Si está listo para disparar: Efecto de parpadeo BLANCO RÁPIDO.
            // (int)(m_blinkTimer.get() * 5) % 2 crea un salto de parpadeo cada 0.2 segundos exactos.
            if ((int)(m_blinkTimer.get() * 5) % 2 == 0) {
                setAllColor(255, 255, 255); // Color Luz Blanca Pura (R,G,B)
            } else {
                setAllColor(0, 0, 0); // Apagar Luz
            }
        } else if (m_matchState.isHubActive()) {
            // EFECTO 2: "PODEMOS ANOTAR - BUSCANDO"
            // Color base: VERDE (Significa meta disponible)
            if (m_matchState.getTimeRemainingInShift() <= 5.0) {
                 // Si nos restan menos de 5 segundos de gracia, empezamos a destellar fuertemente en ráfagas (nerviosismo)
                 if ((int)(m_blinkTimer.get() * 10) % 2 == 0) {
                    setAllColor(0, 255, 0); // Verde sólido
                 } else {
                    setAllColor(0, 0, 0);
                 }
            } else {
                // Hay mucho tiempo: Verde relajado permanente.
                setAllColor(0, 255, 0); 
            }
        } else {
            // EFECTO 3: "NO TIRES AHORITA"
            // Color Fijo ROJO (FMS dice que este cubo Hub no anotará puntos ahorita).
            setAllColor(255, 0, 0);
        }

        // Refresca la memoria física LED con todo el arreglo iterativo para imprimir.
        m_led.setData(m_ledBuffer);
    }

    /**
     * Utilidad (ayudante interno) super rápida para llenar todos los pixeles simultaneamente de la misma tonalidad.
     */
    private void setAllColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b); // Red, Green, Blue
        }
    }
}
