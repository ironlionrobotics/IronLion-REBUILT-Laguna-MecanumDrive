package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

/**
 * Gemelo optimizado directo de AdaptiveShootCommand pero *SOLO* creado para rutinas Autónomas en **PathPlanner**.
 * La diferencia abismal radica en que este comando anula e ignora las órdenes de rotación (NO intercepta ni pide uso de DriveSubsystem).
 * En autónomo, el software `AutoBuilder` de Pathplanner se encargará fluidamente de rotar al chasis sobre la ruta (Holonomic Splines);
 * `AutoShootCommand` actuará como un Parásito inteligente (Parallel Overlapping) que espera a estar a tiempo real cerca y solo aprieta el gatillo.
 */
public class AutoShootCommand extends SequentialCommandGroup {
    public AutoShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        // Enlaza la fábrica de acciones en secuencia temporal estricta de orden vertical
        addCommands(
            
            // ------------------------------------------
            // FASE 1: CALENTAMIENTO PREDICTIVO (Overlapping)
            // ------------------------------------------
            deadline(
                // "Terminaré mi tarea en cuanto logremos ver el cartón de frente Y lleguemos a las 4400 RPM para esta coordenada XYZ lejana que calculé" 
                new WaitUntilCommand(() -> shooter.isShooterAtSpeed(vision.getDistanceToTarget()) && vision.hasTarget()),
                // Mientras se desespera esperando lo de arriba, hace circular corriente preventiva
                run(() -> shooter.runShooterAtDistance(vision.getDistanceToTarget()), shooter)
            ),
            
            // ------------------------------------------
            // FASE 2: TIRO EN MILISEGUNDOS!
            // ------------------------------------------
            // Su duración es menor al tiro Humano del teleoperado (Ej: 1.5 seg en vez de 2) 
            // puesto que en Autonomo queremos brincar y escapar despavoridos por más notas al terminar rapidísimo
            deadline(
                new edu.wpi.first.wpilibj2.command.WaitCommand(1.5),
                run(() -> {
                    shooter.runShooterAtDistance(vision.getDistanceToTarget());
                    shooter.runBeltIndexer(); // Arrancar sub-motores que empujan
                    shooter.runFeeder();
                }, shooter)
            ),
            
            // ------------------------------------------
            // FASE 3: SILENCIO DE ARMAMENTO
            // ------------------------------------------
            // Ejecución segura atónica y mortal (Se destruye rápido dejando huérfano para continuar la ruta y gastar memoria CERO en Java).
            runOnce(() -> {
                shooter.stopShooter();
                shooter.stopBeltIndexer();
                shooter.stopFeeder();
            }, shooter)
        );
    }
}
