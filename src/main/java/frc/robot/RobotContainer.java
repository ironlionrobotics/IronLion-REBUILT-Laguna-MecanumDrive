package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private final CommandXboxController m_playjoystickDriverController = new CommandXboxController(DriveConstants.kPlayJoystick_Cool_Port);
    
    private final SlewRateLimiter m_strafeLimiter = new SlewRateLimiter(.5); 

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
        NamedCommands.registerCommand("StartShooter", m_ShooterSubsystem.runShooterCommand());


        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // DEFAULT DRIVE COMMAND (Xbox Controller)
// DEFAULT DRIVE COMMAND (Xbox Controller)
        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                        double deadZone = 0.10; 
                        
                        double avanzar = MathUtil.applyDeadband(-m_playjoystickDriverController.getLeftY(), deadZone) * 1.25; 
                        
                        double lateral = MathUtil.applyDeadband(m_playjoystickDriverController.getLeftX(), deadZone)*1.25;
                        // double lateral = m_strafeLimiter.calculate(rawLateral) * 1.25;
                        
                        double rotate = MathUtil.applyDeadband(m_playjoystickDriverController.getRightX(), deadZone)*1.25;

                        // --- NEW: ROBOT-ORIENTED MATH ---
                        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
                            new edu.wpi.first.math.kinematics.ChassisSpeeds(
                                avanzar, 
                                lateral,
                                rotate
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

        // sequencial commands
        m_joystickMechanismsController.b()
            .onTrue(
                Commands.sequence(
                    m_ShooterSubsystem.runShooterCommand(),

                    new WaitCommand(2), // time to get shooter's inertia 

                    Commands.repeatingSequence(

                        Commands.parallel(
                            m_IntakeSubsystem.runIntakeCommand(),
                            m_ShooterSubsystem.runIndexerAndFeederCommand()
                        )
                        .withTimeout(2), // parar DESPUES de 5 seg

                        Commands.parallel(
                            m_IntakeSubsystem.stopIntakeCommand(),
                            m_ShooterSubsystem.stopIndexerAndFeederCommand()
                        ),
                        new WaitCommand(1)
                    )
                )
            )
            .onFalse(
                Commands.parallel(
                    // 1. Combine both Shooter stops into a single command block
                    Commands.runOnce(() -> {
                        m_ShooterSubsystem.stopShooter();
                        m_ShooterSubsystem.stopIndexerAndFeeder();
                    }, m_ShooterSubsystem), // Require the subsystem once here!
                    
                    // 2. Stop the Intake (Different subsystem, so parallel is safe)
                    m_IntakeSubsystem.stopIntakeCommand()
                )
            );

  
        // reset pose
        m_playjoystickDriverController.start()
            .onTrue(Commands.runOnce(() -> m_DriveSubsystem.resetOdometry(new Pose2d())));
            
        // --- LIMELIGHT AUTO AIM ONLY ROTATE ---
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

        m_joystickMechanismsController.povUp() //set shooter speed to 1.0
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = 1.0;
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));

        // D-Pad RIGHT: Increment by 0.2 (Max limit of 1.0)
        m_joystickMechanismsController.povRight()
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.min(1.0, TunableConstants.shooterSpeed + 0.2);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));

        // D-Pad LEFT: Decrement by 0.2 (Min limit of 0.0)
        m_joystickMechanismsController.povLeft()
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.max(0.0, TunableConstants.shooterSpeed - 0.2);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));
        new Trigger(() -> m_ShooterSubsystem.isAtSpeed() && m_VisionSubsystem.hasTarget())
            .whileTrue(Commands.run(() -> m_playjoystickDriverController.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
            .whileFalse(Commands.runOnce(() -> m_playjoystickDriverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}