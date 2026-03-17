package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunableConstants;
import frc.robot.Commands.RunIntakeCommand;
import frc.robot.Commands.AdaptiveShootCommand;
import frc.robot.Commands.AutoShootCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.MatchStateSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.DashboardSubsystem;

/**
 * RobotContainer: La columna vertebral del robot.
 * Crea subsistemas, registra NamedCommands, y delega los controles a OI.java.
 * 
 * NOTA: Los bindings de botones están en OI.java para facilitar cambios rápidos.
 */
public class RobotContainer {
    // =========================================================================
    // SUBSISTEMAS (Componentes lógicos del robot)
    // =========================================================================
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_DriveSubsystem);
    private final MatchStateSubsystem m_MatchStateSubsystem = new MatchStateSubsystem();
    private final LEDSubsystem m_LedSubsystem = new LEDSubsystem(m_MatchStateSubsystem);
    private final DashboardSubsystem m_DashboardSubsystem = new DashboardSubsystem(m_DriveSubsystem, m_MatchStateSubsystem, m_VisionSubsystem, m_ShooterSubsystem);

    // Selectores y Controles
    public SendableChooser<Command> autoChooser;
    
    // Controles de la Driver Station (Puertos 0 y 2 definidos en Constants)
    private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kJoystickPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(DriveConstants.kJoystick_Cool_Port);

    public RobotContainer() {
        // --- Publicar valores tunables al SmartDashboard ---
        TunableConstants.publishDefaults();

        // --- Registrar NamedCommands para PathPlanner ---
        NamedCommands.registerCommand("runIntake", new RunIntakeCommand(m_IntakeSubsystem, 4));
        NamedCommands.registerCommand("intakePiece", m_IntakeSubsystem.runIntakeCommand());
        NamedCommands.registerCommand("intakeDown", m_IntakeSubsystem.runIntakeElevarCommand(0.1));
        NamedCommands.registerCommand("intakeUp", m_IntakeSubsystem.runIntakeElevarCommand(-0.1));
        NamedCommands.registerCommand("shootHub", new AdaptiveShootCommand(m_ShooterSubsystem, m_VisionSubsystem, m_DriveSubsystem, m_LedSubsystem));
        NamedCommands.registerCommand("autoShoot", new AutoShootCommand(m_ShooterSubsystem, m_VisionSubsystem));
        
        // --- Configurar PathPlanner ---
        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // --- Comando por defecto del Chasis (Manejo Manual con velocidad tunable) ---
        m_DriveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Leer la velocidad actual desde TunableConstants (ajustable en vivo)
                    double speed = TunableConstants.driveSpeedMultiplier;

                    double avanzar = -edu.wpi.first.math.MathUtil.applyDeadband(
                        m_driverController.getLeftY(), DriveConstants.kJoystickDeadband) * speed; 
                    double lateral = -edu.wpi.first.math.MathUtil.applyDeadband(
                        m_driverController.getLeftX(), DriveConstants.kJoystickDeadband) * speed;
                    double rotate = -edu.wpi.first.math.MathUtil.applyDeadband(
                        m_driverController.getRightX(), DriveConstants.kJoystickDeadband) * speed;

                    edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
                        edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                            avanzar, lateral, rotate,
                            m_DriveSubsystem.getGyro().getRotation2d()
                        );

                    m_DriveSubsystem.driveRobotRelative(velocidades);
                },
                m_DriveSubsystem
            )   
        );
        
        // --- Delegar bindings de botones a OI.java ---
        OI.configureBindings(
            m_driverController,
            m_operatorController,
            m_DriveSubsystem,
            m_ShooterSubsystem,
            m_IntakeSubsystem,
            m_VisionSubsystem,
            m_MatchStateSubsystem,
            m_LedSubsystem
        );
    }
    private void configureBindings() {
        // --- SHOOTER (A Button) ---
        m_joystickMechanismsController.a()
            .onTrue(m_shooterSubsystem.setShooterRPMCommand(2000)) // sets shooter toa target of 3000 RPM
            .onFalse(m_shooterSubsystem.stopShooterCommand());

        // --- INDEXER & FEEDER (Left Bumper) ---
        m_joystickMechanismsController.leftBumper()
            .onTrue(m_shooterSubsystem.runIndexerAndFeederCommand())
            .onFalse(m_shooterSubsystem.stopIndexerAndFeederCommand());

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}