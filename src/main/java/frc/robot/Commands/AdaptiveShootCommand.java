package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

/**
 * ¡Comando Avanzado Integrado! (La Torreta Inteligente o "Aim-Bot")
 * Un SequentialCommandGroup une internamente múltiples "mini-comandos", procesándolos 
 * mágicamente uno detrás de otro de forma secuencial y paralela segura dentro del Scheduler Principal.
 * 
 * Efecto Final de esto: Apretamos UN botón humanoide asimétrico (El gatillo), 
 * el software congela el movimiento, apunta al HUB, lee a distancia, carga RPMs, y dispara ráfagas.
 */
public class AdaptiveShootCommand extends SequentialCommandGroup {
    public AdaptiveShootCommand(ShooterSubsystem shooter, VisionSubsystem vision, DriveSubsystem drive, LEDSubsystem leds) {
        
        // Carga internamente la rutina requerida para mover el Chasis
        AutoAlignCommand align = new AutoAlignCommand(drive, vision);
        
        // .addCommands() empaquetará rutinas verticalmente en cronología base (Paso A, y luego de que muera, sigue al Paso B)
        addCommands(
            
            // ----------------------------------------------------------------------------------------------------------
            // PASO 1 (FASE PRE-TIRO): ALINEACION LATERAL Y ARRANQUE DE TURBINAS SIMULTÁNEO
            // ----------------------------------------------------------------------------------------------------------
            // Modificador `deadline`: Realiza tareas masivas (ejecutar varios Runnables al mismo instante en ramales temporales de hilos), 
            // PERO cerrará por completo todo este bloque grupal cuando EL PRIMER TIPO se reporte vencido "¡Acabé la mía!".
            deadline(
                // COMANDO LIDER MUERTE: Un WaitUntil. Esperará un siglo hasta que la subrutina "Alinear" esté en Verde Óptico && 
                // la lectura cruda de RPM cumpla las RPM dictadas por los Metros pitagóricos actuales a la meta. 
                new WaitUntilCommand(() -> align.isAligned() && shooter.isShooterAtSpeed(vision.getDistanceToTarget())),
                
                // MIENTRAS tanto, se correrán estas sub-actividades hasta que muera la línea superior:
                align, // Mueve activamente las llantas usando PID (Fase Autónoma Local Parcial Mágica)
                run(() -> shooter.runShooterAtDistance(vision.getDistanceToTarget()), shooter) // Acelera en falso las llantas de fuego
            ),

            // ----------------------------------------------------------------------------------------------------------
            // PASO 2 (FASE DE IMPACTO): ALIMENTAR EL CARTUCHO
            // ----------------------------------------------------------------------------------------------------------
            // Llegamos aquí significa que la espera de arriba culminó (Tenemos target fijo e inercias optimizadas super-duras)
            deadline(
                // Lider Deadline: Se esperará en un hilo dormido 2.0 gloriosos segundos...
                new edu.wpi.first.wpilibj2.command.WaitCommand(2.0),
                
                // MIENTRAS ESPERA 2.0s: El robot ejecuta esta lambda para mandar a encender con toda furia las refacciones internas.
                // Como las llantas principales (shooter) ya venían encendidas por RPM del PID del Paso 1 en la fase de 'calentado', no sufrirá caída masiva de torque inicial.
                run(() -> {
                    shooter.runShooterAtDistance(vision.getDistanceToTarget()); // Mantener el RPM deseado mientras se traga el juguete
                    shooter.runBeltIndexer(); // Sube la malla principal ("Elevador Interno de repuestos")
                    shooter.runFeeder();      // Rueda que roza a alta fuerza contra el cañon. 
                }, shooter)
            ),

            // ----------------------------------------------------------------------------------------------------------
            // PASO 3 (FASE CÚLMINE EXITOSA/ABORTADA): APAGÓN MASIVO CONTROLADO
            // ----------------------------------------------------------------------------------------------------------
            // Acabaron los 2.0s? Perfecto. Mandarlos a volar para cuidar frenos electrodinámicos de batería y apagar voltaje sin frenarlo en seco.
            runOnce(() -> {
                shooter.stopShooter();
                shooter.stopBeltIndexer();
                shooter.stopFeeder();
            }, shooter)
        );
    }
}
