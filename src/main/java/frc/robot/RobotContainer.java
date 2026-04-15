package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunableConstants;
import frc.robot.Commands.FollowTagCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    
    // Unified Single Driver Controller
    private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort);
    
    public SendableChooser<Command> autoChooser;
    public SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();
    
    public RobotContainer() {
        configureBindings();
        Constants.Telemetry.publishDefaults();
        
        // --- FIELD RELATIVE TOGGLE ---
        fieldRelativeChooser.setDefaultOption("Field Relative (Gyro)", true);
        fieldRelativeChooser.addOption("Robot Relative", false);
        SmartDashboard.putData("Drive Mode", fieldRelativeChooser);

        Command autoShootSequence = Commands.sequence(
            m_ShooterSubsystem.runShooterCommand(),
            new WaitCommand(2.0), 
            m_ShooterSubsystem.runIndexerAndFeederCommand(),
            new WaitCommand(0.5),
            Commands.runOnce(() -> {
                m_ShooterSubsystem.stopShooter();
                m_ShooterSubsystem.stopIndexerAndFeeder();
            }, m_ShooterSubsystem) 
        );

        Command indexerSequence = Commands.sequence(
            m_ShooterSubsystem.runIndexerAndFeederCommand(), 
            new WaitCommand(10), 
            m_ShooterSubsystem.stopIndexerAndFeederCommand() 
        );
            
        Command armDownAndIntake = Commands.sequence(
            m_IntakeSubsystem.runIntakeArmReverseCommand(), 
            new WaitCommand(1), 
            Commands.runOnce(() -> {
                m_IntakeSubsystem.runIntake();
                m_IntakeSubsystem.stopIntakeArm();
            }, m_IntakeSubsystem)
        );
            
        // --- PATHPLANNER NAMED COMMANDS ---
        NamedCommands.registerCommand("ArmDownAndIntake", armDownAndIntake);
        NamedCommands.registerCommand("ShootAndStop", autoShootSequence);
        NamedCommands.registerCommand("IndexerAndFeeder", indexerSequence);
        NamedCommands.registerCommand("StopIntake", m_IntakeSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("ArmElevate", m_IntakeSubsystem.runIntakeArmCommand());
        NamedCommands.registerCommand("ArmDown", m_IntakeSubsystem.runIntakeArmReverseCommand());
        NamedCommands.registerCommand("StopArm", m_IntakeSubsystem.stopIntakeArmCommand());
        NamedCommands.registerCommand("StopShooter", m_ShooterSubsystem.stopShooterCommand());
        NamedCommands.registerCommand("StartShooter", m_ShooterSubsystem.runShooterCommand());

        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // --- DEFAULT DRIVE COMMAND ---
        m_DriveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    // WPILib Conventions: Left Y is Forward (Negative raw Y is forward), Left X is Strafe (Negative raw X is left)
                    double deadband = 0.10; 
                    double avanzar = MathUtil.applyDeadband(-m_driverController.getLeftY(), deadband) * 2; 
                    double lateral = MathUtil.applyDeadband(m_driverController.getLeftX(), deadband) * 2;
                    double rotate = MathUtil.applyDeadband(m_driverController.getRightX(), deadband) * 2;

                    ChassisSpeeds velocidades;

                    // Toggle based on Dashboard Chooser
                    if (fieldRelativeChooser.getSelected()) {
                        velocidades = ChassisSpeeds.fromFieldRelativeSpeeds(avanzar, lateral, rotate, m_DriveSubsystem.getHeading());
                    } else {
                        velocidades = new ChassisSpeeds(avanzar, lateral, rotate);
                    }

                    m_DriveSubsystem.driveRobotRelative(velocidades);
                },
                m_DriveSubsystem
            )   
        );

    }

    private void configureBindings() {
        // --- INTAKE (Left Trigger) ---
        m_driverController.leftTrigger().and(m_driverController.leftBumper().negate())
           .onTrue(
                Commands.parallel(
                    m_IntakeSubsystem.runIntakeCommand(), 
                    m_ShooterSubsystem.runShooterCommand()
                )
            )
            .onFalse(
                Commands.parallel(
                    m_IntakeSubsystem.stopIntakeCommand(), 
                    m_ShooterSubsystem.stopShooterCommand()

                ));
// --- SHOOTER (Right Trigger) ---
        m_driverController.rightTrigger()
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

// --- REVERSE INTAKE (X Button) ---
        //m_driverController.x()
        //    .onTrue(m_IntakeSubsystem.runIntakeReverseCommand())
        //    .onFalse(m_IntakeSubsystem.stopIntakeCommand());
// TODO:
        m_driverController.x()
            .whileTrue(new FollowTagCommand(m_DriveSubsystem));

// --- REVERSE SHOOTER (Y Button) ---
        m_driverController.y()
            .onTrue(m_ShooterSubsystem.runShooterReverseCommand())
            .onFalse(m_ShooterSubsystem.stopShooterCommand());

// --- REVERSE FEEDER & INDEXER (B Button) ---
        m_driverController.b()
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederReverseCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

// --- INTAKE ARM UP (Right Bumper) ---
        m_driverController.leftBumper()
            .onTrue(m_IntakeSubsystem.runIntakeArmCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand());

// --- INTAKE ARM DOWN (Left Bumper) ---
        m_driverController.rightBumper()
            .onTrue(m_IntakeSubsystem.runIntakeArmReverseCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand());

// --- ADJUST SHOOTER SPEED (D-Pad) ---
        m_driverController.povUp()
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.min(1.0, TunableConstants.shooterSpeed + 0.1);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));
            
        m_driverController.povDown() 
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.max(0.6, TunableConstants.shooterSpeed - 0.1);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));
        m_driverController.back()
            .onTrue(Commands.runOnce(
                () -> m_DriveSubsystem.zeroHeading(), 
                m_DriveSubsystem
            ));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Accessors for Telemetry
    public DriveSubsystem getDrive() { return m_DriveSubsystem; }
    public IntakeSubsystem getIntake() { return m_IntakeSubsystem; }
    public ShooterSubsystem getShooter() { return m_ShooterSubsystem; }
}