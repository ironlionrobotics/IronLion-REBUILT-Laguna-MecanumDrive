package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.RunIntakeCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
public class RobotContainer {
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    private final CommandXboxController m_joystickMechanismsController = new CommandXboxController(DriveConstants.kJoystickPort);
    private final CommandJoystick m_joystickDriverController = new CommandJoystick(DriveConstants.kJoystick_Cool_Port);

    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        // ...
        // Build an auto chooser. This will use Commands.none() as the default option.
        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // 4. CONFIGURAR COMANDO POR DEFECTO (MANEJO)
        // Esto reemplaza lo que tenías en teleopPeriodic en Robot.java
        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                        double deadZone = 0.1;
                        double avanzar = -MathUtil.applyDeadband(m_joystickDriverController.getY(), deadZone) * 3.0; 
                        double lateral = -MathUtil.applyDeadband(m_joystickDriverController.getX(), deadZone) * 3.0;
                        double rotate = -MathUtil.applyDeadband(m_joystickDriverController.getTwist(), deadZone) * 3.0;

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
        //TODO: UNCOMMENT CONFIGURE BINDINGS
       // configureBindings();
    }
    private void configureBindings() {
        // --- SHOOTER (A Button) ---
        m_joystickMechanismsController.a()
            .onTrue(m_shooterSubsystem.setShooterRPMCommand(3000)) // Command that sets shooter to 3000 RPM
            .onFalse(m_shooterSubsystem.stopShooterCommand());

        // --- INDEXER & FEEDER (Left Bumper) ---
        m_joystickMechanismsController.leftBumper()
            .onTrue(m_shooterSubsystem.runIndexerAndFeederCommand())
            .onFalse(m_shooterSubsystem.stopIndexerAndFeederCommand());

        // --- INTAKE (X Button) ---
        // Assuming you have runReverseIntakeCommand() in your subsystem that sets it to -1
        m_joystickMechanismsController.x()
            .onTrue(m_intakeSubsystem.runIntakeCommand())
            .onFalse(m_intakeSubsystem.stopIntakeCommand());
        // --- ARM ELEVATION (Triggers) ---
        // triggers axis (0.0 to 1.0)
        new Trigger(() -> m_joystickMechanismsController.getRightTriggerAxis() == 1.0)
            .onTrue(m_intakeSubsystem.setArmAngleCommand(120)) // Command that sets motor to -0.1
            .onFalse(m_intakeSubsystem.setArmAngleCommand(0)); // Command that sets motor to 0
    }
    //TODO: UNCOMMENT CONFIGURE BINDINGS
   //private void configureBindings() {
   //    new JoystickButton(m_driverController, XboxController.Button.kA.value)
   //        .whileTrue(m_DriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

   //    // Botón B: Dynamic Forward (Salto brusco hacia adelante) -> Para calcular kA
   //    new JoystickButton(m_driverController, XboxController.Button.kB.value)
   //        .whileTrue(m_DriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
   //    // Botón X: Quasistatic Reverse (Rampa suave hacia atrás)
   //    new JoystickButton(m_driverController, XboxController.Button.kX.value)
   //        .whileTrue(m_DriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

   //    // Botón Y: Dynamic Reverse (Salto brusco hacia atrás)
   //    new JoystickButton(m_driverController, XboxController.Button.kY.value)
   //        .whileTrue(m_DriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
   //}

    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
    }

}