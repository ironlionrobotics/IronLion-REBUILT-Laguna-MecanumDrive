package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

/**
 * ¡El Aim-Bot Original de Teleop! Automáticamente gira de manera rápida la robusta 
 * base del robot de 100 libras para mantener anclada y asegurada en el centro
 * de nuestra cámara inteligente Limelight al luminoso objetivo fijado.
 * Transforma el robot de un carrito RC a una Torreta Dinámica sobre ruedas Mecanum.
 */
public class AutoAlignCommand extends Command {
    // Declaración de los componentes base sobre los cuales se operará (Sub-Esclavos Ocupados)
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    
    // Controlador Físico-Matemático Predictor del Error de Ángulo Visual Continuo (PID)
    // Calcula qué tanta fuerza de voltaje extra (Rotación Angular rápida/lenta) necesita aplicar
    // dinámicamente cada centésima de segundo a los motores Sparks sabiendo qué tan cerca o lejos
    // se le sigue cruzando o escurriendo la gráfica de HUB a nuestra imagen proyectada en cámara inteligente.
    private final PIDController m_controller;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        m_driveSubsystem = drive;
        m_visionSubsystem = vision;
        
        m_controller = new PIDController(VisionConstants.kAlignP, VisionConstants.kAlignI, VisionConstants.kAlignD);
        
        // Indicadores métricos para Java en los que le decimos qué tantos grados (Ej. 0.5) de desajuste son aceptables 
        // para dar por decretado y concluido el encuadre exitoso a verde de disparo inminente final. 
        m_controller.setTolerance(VisionConstants.kAlignTolerance);
        
        // ¡PROTECCIÓN CRUCIAL!: "addRequirements" le dice muy estricto al Sistema del Robot: 
        // "¡Este Comando SECUESTRA en exclusividad total todo el control del Chasis de movimiento!"
        // Si piloto Humano (Joysticks) chocase en dedo mandando moverse y a la vez oprimir Alineado Automático veloz, 
        // WPILib ignorará con preferencia a uno de los dos y cortará voltajes mezclados. Previene muertes de hardware extrañísimas. 
        addRequirements(m_driveSubsystem);
    }

    /** 
     * EL CORAZÓN LÚPICO DE EJECUCIÓN (Latir). Se reanima solo 50 veces por cada segundo activo en tiempo real 
     * que transcurra con su botón encendido. (Cada 20 Microsegundos recalcula matemáticas de encuadre). 
     */
    @Override
    public void execute() {
        if (m_visionSubsystem.hasTarget()) {
            
            // "Apúntalo Crítico": Forzamos a que nuestra Variable Independiente de error TX(X de cámara cruzada de Visión) aspire llegar a "0.0".
            // Y de la gran diferencia de la fórmula saldrá un Velocímetro decimal Angular brutal...
            double rotationSpeed = m_controller.calculate(m_visionSubsystem.getTx(), 0);
            
            // Re-transmisión inyectada directamente pidiendo Volts Brutos y Letales al hardware del Chasis en vivo.
            // Componentes X, Y y Omega.
            // -> X (0 m/s Adelante) 
            // -> Y (0 m/s Caminar Cangrejo a los lados)
            // -> OMEGA (Girado tipo huracán sobre nuestro centro de eje ciego robotico exigiendo nuestro 'rotationSpeed')
            m_driveSubsystem.driveRobotRelative(
                new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, rotationSpeed)
            );
        } else {
            // Protección Antifallas Visión: ¿Alguien cubrió o tapó el lente?  
            // Se envía de inmediato un ChasisSpeeds en Blanco sin valores que forzará al instante Frenos Electromagnéticos seguros.
            m_driveSubsystem.driveRobotRelative(new edu.wpi.first.math.kinematics.ChassisSpeeds());
        }
    }

    /**
     * Termina el trayecto histórico virtual (Suicide Event) al detectar cese de exigencias (Soltar Botón o Choque de Requerimientos).
     */
    @Override
    public void end(boolean interrupted) {
        // Cerrado con Candado Mágico: Mando la petición base general sin información vectorial de torque para no arrastrar inercialmente una patinada.
        m_driveSubsystem.driveRobotRelative(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    }

    /**
     * Una métrica pública interior encubierta que actúa como un Informante... 
     * Rutinas en paralelo masivas compuestas como la reina 'AdaptiveShootCommand' la utilizan en un bucle temporal
     * para cuestionarle: Oye, Hermano Comando AutoAlign ¿Me autorizas informar que la puntería ya se completó bien fina?
     * Y ella al responder su boolean mágico "TRUE", desbloquea compuertas de llantas para disparo infernal de FUEL. 
     */
    public boolean isAligned() {
        // Se valida estricto ambas lógicas dependendientes primordiales visuales: ¿Reconozco un cubo verde AND (y lógico) no excedo los .5 de error tolerables PID?
        return m_visionSubsystem.hasTarget() && m_controller.atSetpoint();
    }
}
