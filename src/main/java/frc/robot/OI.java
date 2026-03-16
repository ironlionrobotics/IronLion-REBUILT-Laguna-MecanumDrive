package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AdaptiveShootCommand;
import frc.robot.Subsystems.*;

/**
 * OI (Operator Interface) - Mapeo de botones de la Driver Station.
 * Ahora optimizado para DOS PILOTOS (Driver y Operator) buscando eficiencia y Cero Carga Cognitiva.
 */
public class OI {

    public static void configureBindings(
        CommandXboxController driver,
        CommandXboxController operator,
        DriveSubsystem drive,
        ShooterSubsystem shooter,
        IntakeSubsystem intake,
        VisionSubsystem vision,
        MatchStateSubsystem matchState,
        LEDSubsystem leds
    ) {
        // =============================================================
        //  🎮 CONTROL 1: EL PILOTO (DRIVER) - "Mover y Tragar"
        // =============================================================
        // Su única misión en la competencia es posicionarse, evadir defensa 
        // y pasar por encima de las piezas en el piso para asegurar la munición.

        // --- INTAKE (Recolector Frontal) ---
        // Gatillo Derecho (RT): Tragar pieza (Succionar) - Acción Frecuente
        driver.rightTrigger().whileTrue(intake.runIntakeCommand());
        
        // Gatillo Izquierdo (LT): Expulsar pieza atorada de emergencia
        driver.leftTrigger().whileTrue(intake.runIntakeReverseCommand());
        
        // Bumpers (RB/LB): Bajar y Subir el brazo articulado del Intake para despejar el camino
        driver.rightBumper().whileTrue(intake.runIntakeElevarCommand(0.1)); // Bajar/Sacar intake
        driver.leftBumper().whileTrue(intake.runIntakeElevarCommand(-0.1)); // Subir/Guardar intake

        // --- NAVEGACIÓN ADAS ("Piloto Automático de Emergencia/Ciego") ---
        // Botón Y: Rinde el chasis al PathPlanner para viajar esquiando directo al HUB azul.
        driver.y().whileTrue(
            AutoBuilder.pathfindToPose(
                Constants.FieldConstants.m_blueHubPose, 
                new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)), 
                0.0 
            )
        );

        // --- TUNEOS DE INGENIERÍA PARA FÍSICA DE CHASIS (Solo en pits) ---
        driver.povUp().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driver.povRight().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver.povDown().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driver.povLeft().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        // =============================================================
        //  🎮 CONTROL 2: EL COPILOTO (OPERATOR / GUNNER) - "Apuntar, Matar y Colgarse"
        // =============================================================
        // Su misión es gestionar la munición guardada, disparar bajo presión o de manera inteligente,
        // administrar los últimos 20 segundos de EndGame (Climber) y salvar al robot de fallas de cámara.

        // --- DISPARO ADAPTATIVO (La Torreta Aim-Bot) ---
        // Gatillo Derecho (RT) del Copiloto: Solicita interrupción parcial del chasis, apunta automático a cámara y dispara según métrica interpolada.
        operator.rightTrigger().whileTrue(
            new AdaptiveShootCommand(shooter, vision, drive, leds)
        );

        // --- OVERRIDE MANUAL (Por si alguien rompe la cámara) ---
        // Botón B: Disparo manual sordo. Solo prende el shooter en base a la Velocidad Tunable del SmartDashboard.
        operator.b()
            .onlyIf(() -> matchState.isHubActive() && matchState.getTimeRemainingInShift() > 2.0)
            .whileTrue(shooter.runShooterCommand());
            
        // Botón X: Empujar pieza bruscamente hacia el shooter cuando el copiloto considere adecuado (Feed).
        operator.x()
            .onlyIf(() -> matchState.isHubActive() && matchState.getTimeRemainingInShift() > 2.0)
            .whileTrue(
                shooter.runBeltIndexerCommand().alongWith(shooter.runFeederCommand())
            );

        // --- ESCALADOR (ENDGAME - RESERVAS: FASE 8) ---
        // El D-Pad del Operador estará destinado estrictamente para los winches de escalar en la fase final.
        // Ej: operator.povUp().whileTrue(climber.extendArmsCommand());
        // Ej: operator.povDown().whileTrue(climber.retractArmsAndLiftCommand());

        // =============================================================
        //  SEGURIDAD COMPARTIDA
        // =============================================================
        // Botón BACK/SELECT en ambos controles actúa como KILL SWITCH (Freno Seco Paranoico)
        Command killSwitch = Commands.runOnce(() -> {
            shooter.stopShooter();
            shooter.stopBeltIndexer();
            shooter.stopFeeder();
            intake.stopIntake();
            intake.stopIntakeElevar();
        });
        driver.back().onTrue(killSwitch);
        operator.back().onTrue(killSwitch);


        // =============================================================
        //  ALERTAS SENSORIALES COMPARTIDAS
        // =============================================================
        // Ambos controles vibrarán emulando un "Latido de Corazón" cuando el Hub se vaya a apagar (Quedan 5 Seg).
        new Trigger(() -> matchState.getTimeRemainingInShift() <= 5.0 && matchState.getTimeRemainingInShift() > 0.1)
            .whileTrue(new RunCommand(() -> {
                double currentSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 10;
                boolean rumblePulse = ((int)currentSec % 5 == 0);
                
                driver.getHID().setRumble(RumbleType.kBothRumble, rumblePulse ? 1.0 : 0.0);
                operator.getHID().setRumble(RumbleType.kBothRumble, rumblePulse ? 1.0 : 0.0);
                
            }).finallyDo(() -> {
                driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            }));

        // Feedback Perimetral a LEDs indicando si "El Aim-bot" de la visión ya está lockeado e ideal para Disparo Certero.
        new Trigger(vision::hasTarget)
            .debounce(0.1)
            .onTrue(new RunCommand(() -> leds.setAlignedToHub(
                Math.abs(vision.getTx()) < Constants.TunableConstants.alignTolerance), 
                leds))
            .onFalse(new RunCommand(() -> leds.setAlignedToHub(false), leds));
    }
}
