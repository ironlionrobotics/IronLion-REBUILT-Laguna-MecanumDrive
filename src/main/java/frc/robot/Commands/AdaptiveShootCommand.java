package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.LEDSubsystem;

public class AdaptiveShootCommand extends SequentialCommandGroup {
    public AdaptiveShootCommand(ShooterSubsystem shooter, VisionSubsystem vision, DriveSubsystem drive, LEDSubsystem leds) {
        AutoAlignCommand align = new AutoAlignCommand(drive, vision);
        
        addCommands(
            // 1. Alinearse y calentar motores en paralelo
            deadline(
                new WaitUntilCommand(() -> align.isAligned() && shooter.isShooterAtSpeed(vision.getDistanceToTarget())),
                align,
                run(() -> shooter.runShooterAtDistance(vision.getDistanceToTarget()), shooter)
            ),
            // 2. Disparar (activar indexer y feeder) durante 2 segundos
            deadline(
                new edu.wpi.first.wpilibj2.command.WaitCommand(2.0),
                run(() -> {
                    shooter.runShooterAtDistance(vision.getDistanceToTarget());
                    shooter.runBeltIndexer();
                    shooter.runFeeder();
                }, shooter)
            ),
            // 3. Apagar todo
            runOnce(() -> {
                shooter.stopShooter();
                shooter.stopBeltIndexer();
                shooter.stopFeeder();
            }, shooter)
        );
    }
}
