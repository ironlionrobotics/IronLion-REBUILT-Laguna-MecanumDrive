package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.RunIntakeCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
public class RobotContainer {
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

    public SendableChooser<Command> autoChooser;
    private final XboxController m_driverController = new XboxController(DriveConstants.kJoystickPort);

    public RobotContainer() {
        // ...
        NamedCommands.registerCommand("runIntake", new RunIntakeCommand(m_IntakeSubsystem, 2));
        // Build an auto chooser. This will use Commands.none() as the default option.
        m_DriveSubsystem.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // 4. CONFIGURAR COMANDO POR DEFECTO (MANEJO)
        // Esto reemplaza lo que tenías en teleopPeriodic en Robot.java
        m_DriveSubsystem.setDefaultCommand(
                new RunCommand(
                    () -> {
                    // 1. Leemos los joysticks (invertimos Y para que adelante sea positivo)
                        double avanzar = -m_driverController.getLeftY() * 3.0; // Multiplica por tu velocidad máxima deseada en m/s
                        double lateral = -m_driverController.getLeftX() * 3.0;
                        double rotate = -m_driverController.getRightX() * 3.0;

                        // 2. Aplicamos cinemática Field-Oriented con el giroscopio
                        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades = 
                            edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                                avanzar, 
                                lateral, 
                                rotate, 
                                DriveSubsystem.m_gyro.getRotation2d()
                            );

                        // 3. Movemos el robot
                        m_DriveSubsystem.driveRobotRelative(velocidades);
                    },
                    m_DriveSubsystem
                )   
        );
        //TODO: UNCOMMENT CONFIGURE BINDINGS
       // configureBindings();
    }

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