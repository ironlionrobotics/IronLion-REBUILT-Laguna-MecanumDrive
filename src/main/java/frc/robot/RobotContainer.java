package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoAlignCommand;
import frc.robot.Commands.AutoShootCommand;
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
    private final CommandXboxController m_joystickDriverController = new CommandXboxController(DriveConstants.kJoystick_Cool_Port);
    private final CommandXboxController m_playjoystickDriverController = new CommandXboxController(DriveConstants.kPlayJoystick_Cool_Port);

    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        TunableConstants.publishDefaults();

        // --- PATHPLANNER NAMED COMMANDS ---
        NamedCommands.registerCommand("StartIntake", m_IntakeSubsystem.runIntakeCommand());
        NamedCommands.registerCommand("StopIntake", m_IntakeSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("Arm Elevate", m_IntakeSubsystem.runIntakeArmCommand());
        NamedCommands.registerCommand("Arm Down", m_IntakeSubsystem.runIntakeArmReverseCommand());
        NamedCommands.registerCommand("Stop Arm", m_IntakeSubsystem.stopIntakeArmCommand());
        NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(m_ShooterSubsystem, m_VisionSubsystem));
        NamedCommands.registerCommand("StopShooter", m_ShooterSubsystem.stopShooterCommand());

        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // DEFAULT DRIVE COMMAND (Xbox Controller)
        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                        double deadZone = 0.10; // Responsive deadband
                        double avanzar = -MathUtil.applyDeadband(-m_playjoystickDriverController.getLeftY(), deadZone) * 3.0; 
                        double lateral = -MathUtil.applyDeadband(m_playjoystickDriverController.getLeftX(), deadZone) * 5.0;
                        double rotate = -MathUtil.applyDeadband(m_playjoystickDriverController.getRightX(), deadZone) * 3.0;

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

        m_playjoystickDriverController.start()
            .onTrue(Commands.runOnce(() -> m_DriveSubsystem.resetOdometry(new Pose2d())));
            
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
        m_playjoystickDriverController.a()
            .onTrue(m_ClimberSubsystem.runClimberCommand())
            .onFalse(m_ClimberSubsystem.stopClimberCommand());
        
        m_playjoystickDriverController.y()
            .onTrue(m_ClimberSubsystem.runClimberReverseCommand())
            .onFalse(m_ClimberSubsystem.stopClimberCommand());

        // --- AIM BOT / AUTO SHOOT ---
        m_playjoystickDriverController.leftTrigger()    
            .whileTrue(new AutoShootCommand(m_ShooterSubsystem, m_VisionSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}