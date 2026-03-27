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
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_DriveSubsystem);
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    
    private final CommandXboxController m_joystickMechanismsController = new CommandXboxController(DriveConstants.kJoystickPort);
    private final CommandXboxController m_playjoystickDriverController = new CommandXboxController(DriveConstants.kPlayJoystick_Cool_Port);
    
    private final SlewRateLimiter m_strafeLimiter = new SlewRateLimiter(.5); 

    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        TunableConstants.publishDefaults();
        Command autoShootSequence = Commands.sequence(
            // 1. Start the shooter wheels
            m_ShooterSubsystem.runShooterCommand(),
            
            // 2. Wait for the wheels to gain inertia (Adjust this if needed!)
            new WaitCommand(2.0), 
            
            // 3. Fire the indexer and feeder to shove the note into the wheels
            m_ShooterSubsystem.runIndexerAndFeederCommand(),
            
            // 4. Wait for the note to physically leave the robot
            new WaitCommand(0.5),
            
            // 5. Safely turn everything off
            Commands.runOnce(() -> {
                m_ShooterSubsystem.stopShooter();
                m_ShooterSubsystem.stopIndexerAndFeeder();
            }, m_ShooterSubsystem) // Require the subsystem to prevent command clashing
        );

        Command indexerSequence = Commands.sequence(
            m_ShooterSubsystem.runIndexerAndFeederCommand(), // Start the indexer and feeder
            new WaitCommand(10), // Run for 1 second (adjust as needed)
            m_ShooterSubsystem.stopIndexerAndFeederCommand() // Stop the indexer and feeder
            );
            
        Command armDownAndIntake = Commands.sequence(
            m_IntakeSubsystem.runIntakeArmReverseCommand(), // Start the indexer and feeder
            new WaitCommand(1), // Run for 1 second (adjust as needed)
            Commands.runOnce(() -> {
                m_IntakeSubsystem.runIntake();
                m_IntakeSubsystem.stopIntakeArm();
            }, m_IntakeSubsystem
            ));
            
        // --- PATHPLANNER NAMED COMMANDS ---
        NamedCommands.registerCommand("ArmDownAndIntake", armDownAndIntake);
        NamedCommands.registerCommand("ShootAndStop", autoShootSequence);
        NamedCommands.registerCommand("IndexerAndFeeder", indexerSequence);
        NamedCommands.registerCommand("StopIntake", m_IntakeSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("ArmElevate", m_IntakeSubsystem.runIntakeArmCommand());
        NamedCommands.registerCommand("ArmDown", m_IntakeSubsystem.runIntakeArmReverseCommand());
        NamedCommands.registerCommand("StopArm", m_IntakeSubsystem.stopIntakeArmCommand());
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
                        double deadZone = 0.0; 
                        
                        double avanzar = -m_playjoystickDriverController.getLeftX() * 2.95; 
                        
                        double lateral = -m_playjoystickDriverController.getLeftY() * 2.95;
                        // double lateral = m_strafeLimiter.calculate(rawLateral) * 1.25;
                         
                        double rotate = m_playjoystickDriverController.getRightX() * 2.95;

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
        m_joystickMechanismsController.leftBumper().and(m_joystickMechanismsController.rightBumper().negate())
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

        m_joystickMechanismsController.leftBumper().and(m_joystickMechanismsController.rightBumper())
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederReverseCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

        // --- INTAKE ---
        m_joystickMechanismsController.x().and(m_joystickMechanismsController.rightBumper().negate())
            .onTrue(m_IntakeSubsystem.runIntakeCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeCommand());

        // Reverse Intake (Hold Right Bumper AND press X)
        m_joystickMechanismsController.x().and(m_joystickMechanismsController.rightBumper())
            .onTrue(m_IntakeSubsystem.runIntakeReverseCommand())
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
                        .withTimeout(5), // parar DESPUES de 5 seg

                        Commands.parallel(
                            Commands.runOnce(() -> m_IntakeSubsystem.stopIntake(), m_IntakeSubsystem),
                            Commands.runOnce(() -> m_ShooterSubsystem.stopIndexerAndFeeder(), m_ShooterSubsystem)
                        ),

                        new WaitCommand(.5)
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
            
        // --- LIMELIGHT AUTO AIM ONLY ROTATE ---
//        m_playjoystickDriverController.leftBumper()
//            .whileTrue(
//                new AutoAlignCommand(
//                    m_DriveSubsystem, 
//                    m_VisionSubsystem, 
//                    () -> m_playjoystickDriverController.getRightX(),  
//                    () -> -m_playjoystickDriverController.getLeftY() 
//                )
//            );

        // --- INTAKE REVERSE (Left Trigger) ---
        m_playjoystickDriverController.leftTrigger()
            .onTrue(m_IntakeSubsystem.runIntakeReverseCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeCommand());

        // --- INTAKE + SHOOTER COMBO (Right Trigger) ---
        // Uses Commands.parallel to safely run both subsystems at once
        m_playjoystickDriverController.rightTrigger()
            .onTrue(Commands.parallel(
                m_IntakeSubsystem.runIntakeCommand(),
                m_ShooterSubsystem.runShooterCommand()
            ))
            .onFalse(Commands.parallel(
                m_IntakeSubsystem.stopIntakeCommand(),
                m_ShooterSubsystem.stopShooterCommand()
            ));

        // --- FEEDER & INDEXER NORMAL (A Button) ---
        m_playjoystickDriverController.a()
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

        // --- FEEDER & INDEXER REVERSE (B Button) ---
        m_playjoystickDriverController.b()
            .onTrue(m_ShooterSubsystem.runIndexerAndFeederReverseCommand())
            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());

        // --- SHOOTER REVERSE (Y Button) ---
        m_playjoystickDriverController.y()
            .onTrue(m_ShooterSubsystem.runShooterReverseCommand())
            .onFalse(m_ShooterSubsystem.stopShooterCommand());

        // --- INTAKE ARM UP (Left Bumper) ---
        m_playjoystickDriverController.leftBumper()
            .onTrue(m_IntakeSubsystem.runIntakeArmCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand());

        // --- INTAKE ARM DOWN (Right Bumper) ---
        m_playjoystickDriverController.rightBumper()
            .onTrue(m_IntakeSubsystem.runIntakeArmReverseCommand())
            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand());

        m_playjoystickDriverController.povUp() //set shooter speed to 1.0
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.min(1.0, TunableConstants.shooterSpeed + 0.5);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));
        m_playjoystickDriverController.povDown() //set shooter speed to 1.0
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.max(0.2, TunableConstants.shooterSpeed - 0.5);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));

        m_playjoystickDriverController.povRight()
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.min(1.0, TunableConstants.shooterSpeed + 0.1);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));

        // D-Pad LEFT: Decrement by 0.2 (Min limit of 0.0)
        m_playjoystickDriverController.povLeft()
            .onTrue(Commands.runOnce(() -> {
                TunableConstants.shooterSpeed = Math.max(0.4, TunableConstants.shooterSpeed - 0.1);
                SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            }));
        new Trigger(() -> m_VisionSubsystem.hasTarget())
            .whileTrue(Commands.run(() -> m_playjoystickDriverController.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
            .whileFalse(Commands.runOnce(() -> m_playjoystickDriverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}