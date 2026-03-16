package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.RunIntakeCommand;
import frc.robot.Commands.AutoAlignCommand;
import frc.robot.Commands.AdaptiveShootCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.MatchStateSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.DashboardSubsystem;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_DriveSubsystem);
    private final MatchStateSubsystem m_MatchStateSubsystem = new MatchStateSubsystem();
    private final LEDSubsystem m_LedSubsystem = new LEDSubsystem(m_MatchStateSubsystem);
    private final DashboardSubsystem m_DashboardSubsystem = new DashboardSubsystem(m_DriveSubsystem, m_MatchStateSubsystem, m_VisionSubsystem, m_ShooterSubsystem);

    public SendableChooser<Command> autoChooser;
    private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kJoystickPort);

    public RobotContainer() {
        NamedCommands.registerCommand("runIntake", new RunIntakeCommand(m_IntakeSubsystem, 2));
        
        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                        double avanzar = -edu.wpi.first.math.MathUtil.applyDeadband(m_driverController.getLeftY(), DriveConstants.kJoystickDeadband) * 3.0; 
                        double lateral = -edu.wpi.first.math.MathUtil.applyDeadband(m_driverController.getLeftX(), DriveConstants.kJoystickDeadband) * 3.0;
                        double rotate = -edu.wpi.first.math.MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.kJoystickDeadband) * 3.0;

                        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
                            edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                                avanzar, 
                                lateral, 
                                rotate, 
                                m_DriveSubsystem.getGyro().getRotation2d()
                            );

                        m_DriveSubsystem.driveRobotRelative(velocidades);
                    },
                    m_DriveSubsystem
                )   
        );
        configureBindings();
    }

    private void configureBindings() {
        // --- Shooter Control (CHRONOS SYSTEM) --- 
        m_driverController.a()
            .onlyIf(() -> m_MatchStateSubsystem.isHubActive() && m_MatchStateSubsystem.getTimeRemainingInShift() > 2.0)
            .whileTrue(m_ShooterSubsystem.runShooterCommand());
            
        m_driverController.leftBumper()
            .onlyIf(() -> m_MatchStateSubsystem.isHubActive() && m_MatchStateSubsystem.getTimeRemainingInShift() > 2.0)
            .whileTrue(
                m_ShooterSubsystem.runBeltIndexerCommand().alongWith(m_ShooterSubsystem.runFeederCommand())
            );

        // --- Intake Control ---
        m_driverController.x().whileTrue(m_IntakeSubsystem.runIntakeReverseCommand());
        m_driverController.leftTrigger().whileTrue(m_IntakeSubsystem.runIntakeElevarCommand(0.1));
        m_driverController.rightTrigger().whileTrue(m_IntakeSubsystem.runIntakeElevarCommand(-0.1));

        // --- ADAPTIVE SHOOTING (PHASE 5) ---
        m_driverController.rightBumper().whileTrue(
            new AdaptiveShootCommand(m_ShooterSubsystem, m_VisionSubsystem, m_DriveSubsystem, m_LedSubsystem)
        );

        // --- ADAS: Pathfinding ---
        m_driverController.y().whileTrue(
            AutoBuilder.pathfindToPose(
                Constants.FieldConstants.m_blueHubPose,
                new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)), 
                0.0 
            )
        );

        // --- HAPTIC FEEDBACK ---
        new Trigger(() -> m_MatchStateSubsystem.getTimeRemainingInShift() <= 5.0 && m_MatchStateSubsystem.getTimeRemainingInShift() > 0.1)
            .whileTrue(new RunCommand(() -> {
                double currentSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 10;
                if ((int)currentSec % 5 == 0) {
                    m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                } else {
                    m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                }
            }).finallyDo(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

        // --- LED ALIGNMENT FEEDBACK ---
        new Trigger(m_VisionSubsystem::hasTarget)
            .debounce(0.1)
            .onTrue(new RunCommand(() -> m_LedSubsystem.setAlignedToHub(
                Math.abs(m_VisionSubsystem.getTx()) < Constants.VisionConstants.kAlignTolerance), 
                m_LedSubsystem))
            .onFalse(new RunCommand(() -> m_LedSubsystem.setAlignedToHub(false), m_LedSubsystem));

        // --- SysId Routines ---
        m_driverController.povUp().whileTrue(m_DriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_driverController.povRight().whileTrue(m_DriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_driverController.povDown().whileTrue(m_DriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_driverController.povLeft().whileTrue(m_DriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}