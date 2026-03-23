package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoAlignCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunableConstants;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_DriveSubsystem);
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
    
    private final CommandXboxController m_joystickMechanismsController = new CommandXboxController(DriveConstants.kJoystickPort);
    private final CommandJoystick m_joystickDriverController = new CommandJoystick(DriveConstants.kJoystick_Cool_Port);
    private final CommandXboxController m_playjoystickDriverController = new CommandXboxController(DriveConstants.kJoystick_Cool_Port);

    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        TunableConstants.publishDefaults();

        // --- PATHPLANNER NAMED COMMANDS ---
        NamedCommands.registerCommand("StartIntake", m_IntakeSubsystem.runIntakeCommand());
        NamedCommands.registerCommand("StopIntake", m_IntakeSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("RPMShooter", m_ShooterSubsystem.setShooterRPMCommand(2000));
        NamedCommands.registerCommand("Start IndexerAndFeeder", m_ShooterSubsystem.runIndexerAndFeederCommand());
        NamedCommands.registerCommand("StopShooter", m_ShooterSubsystem.stopShooterCommand());
        NamedCommands.registerCommand("Arm Elevate", m_IntakeSubsystem.runIntakeArmCommand());
        NamedCommands.registerCommand("Arm Down", m_IntakeSubsystem.runIntakeArmReverseCommand());
        NamedCommands.registerCommand("Stop Arm", m_IntakeSubsystem.stopIntakeArmCommand());

        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // DEFAULT DRIVE COMMAND
        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                        double deadZone = 0.5; // Fixed deadband to feel responsive
                        double avanzar = -MathUtil.applyDeadband(m_joystickDriverController.getY(), deadZone) * 3.0; 
                        double lateral = -MathUtil.applyDeadband(m_joystickDriverController.getX(), deadZone) * 5.0;
                        double rotate = MathUtil.applyDeadband(m_joystickDriverController.getZ(), deadZone) * 3.0;

                        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
                            edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                                avanzar, 
                                lateral,
                                rotate, 
                                m_DriveSubsystem.getHeading()
                            );

                        m_DriveSubsystem.driveRobotRelative(velocidades);
                    },
                    m_DriveSubsystem
                )   
        );
        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                        double deadZone = 0.2;
                        double avanzar = -MathUtil.applyDeadband(-m_playjoystickDriverController.getLeftY(), deadZone) * 3.0; 
                        double lateral = -MathUtil.applyDeadband(m_playjoystickDriverController.getLeftX(), deadZone) * 5.0;
                        double rotate = MathUtil.applyDeadband(m_playjoystickDriverController.getRightX(), deadZone) * 3.0;

                        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
                            edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                                avanzar, 
                                lateral,
                                rotate, 
                                m_DriveSubsystem.getHeading()
                            );

                        m_DriveSubsystem.driveRobotRelative(velocidades);
                    },
                    m_DriveSubsystem
                )   
        );
    }

    private void configureBindings() {
        // --- SHOOTER ---
        m_joystickMechanismsController.a()
            .onTrue(m_ShooterSubsystem.runShooterCommand())
            .onFalse(m_ShooterSubsystem.stopShooterCommand());

        // --- INDEXER & FEEDER ---
        m_joystickMechanismsController.leftBumper()
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

        // --- INTAKE ---
        m_joystickMechanismsController.x()
            .onTrue(m_IntakeSubsystem.runIntakeCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeCommand());

        // --- ARM ELEVATION ---
        m_joystickMechanismsController.back()
            .onTrue(m_IntakeSubsystem.runIntakeArmReverseCommand()) 
            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand()); 

        m_joystickMechanismsController.start()
            .onTrue(m_IntakeSubsystem.runIntakeArmCommand()) 
            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand()); 
            
        // --- LIMELIGHT AUTO AIM ---
        m_playjoystickDriverController.leftBumper()
            .whileTrue(
                new AutoAlignCommand(
                    m_DriveSubsystem, 
                    m_VisionSubsystem, 
                    () -> -m_playjoystickDriverController.getLeftY(), 
                    () -> m_playjoystickDriverController.getRightX()  
                )
            );
        
        // --- CLIMBER ---
        m_joystickDriverController.button(7)
            .onTrue(m_ClimberSubsystem.runClimberCommand())
            .onFalse(m_ClimberSubsystem.stopClimberCommand());
        
        m_joystickDriverController.button(8)
            .onTrue(m_ClimberSubsystem.runClimberReverseCommand())
            .onFalse(m_ClimberSubsystem.stopClimberCommand());

        m_playjoystickDriverController.a()
            .onTrue(m_ClimberSubsystem.runClimberCommand())
            .onFalse(m_ClimberSubsystem.stopClimberCommand());
        
        m_playjoystickDriverController.y()
            .onTrue(m_ClimberSubsystem.runClimberReverseCommand())
            .onFalse(m_ClimberSubsystem.stopClimberCommand());

        m_playjoystickDriverController.leftTrigger()    
            .whileTrue(m_ShooterSubsystem.autoShootCommand(() -> m_VisionSubsystem.getTy()))
            .onFalse(m_ShooterSubsystem.stopShooterCommand());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}