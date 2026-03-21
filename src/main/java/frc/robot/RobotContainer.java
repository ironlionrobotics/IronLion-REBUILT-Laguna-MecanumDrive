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

import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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



    public SendableChooser<Command> autoChooser;



    public RobotContainer() {

        configureBindings();

        TunableConstants.publishDefaults();



        // Build an auto chooser. This will use Commands.none() as the default option.

        m_DriveSubsystem.configureAutoBuilder();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);



        // 4. CONFIGURAR COMANDO POR DEFECTO (MANEJO)

        // Esto reemplaza lo que tenías en teleopPeriodic en Robot.java

        m_DriveSubsystem.setDefaultCommand(

                new RunCommand(

                    () -> {

                        double deadZone = .25;

                        double avanzar = -MathUtil.applyDeadband(m_joystickDriverController.getY(), deadZone) * 3.0;

                        double lateral = -MathUtil.applyDeadband(m_joystickDriverController.getX(), deadZone) * 5.0;

                        double rotate = MathUtil.applyDeadband(m_joystickDriverController.getZ(), deadZone) * 3.0;



                        edu.wpi.first.math.kinematics.ChassisSpeeds velocidades =

                            edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(

                                //TODO: testing, for now

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

        // --- SHOOTER (A Button) ---

        m_joystickMechanismsController.a()

            .onTrue(m_ShooterSubsystem.setShooterRPMCommand(2000)) // sets shooter toa target of 3000 RPM

            .onFalse(m_ShooterSubsystem.stopShooterCommand());



        // --- INDEXER & FEEDER (Left Bumper) ---

        m_joystickMechanismsController.leftBumper()

            .onTrue(m_ShooterSubsystem.runIndexerAndFeederCommand())

            .onFalse(m_ShooterSubsystem.stopIndexerAndFeederCommand());



        // --- INTAKE (X Button) ---

        // Assuming you have runReverseIntakeCommand() in your subsystem that sets it to -1

        m_joystickMechanismsController.x()

            .onTrue(m_IntakeSubsystem.runIntakeCommand())

            .onFalse(m_IntakeSubsystem.stopIntakeCommand());

                       

       



        // --- ARM ELEVATION (Triggers) ---

        // triggers axis (0.0 to 1.0)



        m_joystickMechanismsController.back()

            .onTrue(m_IntakeSubsystem.runIntakeArmReverseCommand()) // Command that sets motor to -0.1

            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand()); // Command that sets motor to 0



        m_joystickMechanismsController.start()

            .onTrue(m_IntakeSubsystem.runIntakeArmCommand()) // Command that sets motor to the set spee

            .onFalse(m_IntakeSubsystem.stopIntakeArmCommand()); // Command that sets motor to 0

       

           

        // LIMELIGHT AUTO AIM

        m_joystickDriverController.button(1)

            .whileTrue(

                new AutoAlignCommand(

                    m_DriveSubsystem,

                    m_VisionSubsystem,

                    () -> m_joystickDriverController.getY(), // Avanzar supplier

                    () -> m_joystickDriverController.getX()  // Lateral supplier

                )

            );

       

        m_joystickDriverController.button(7)

            .onTrue(m_ClimberSubsystem.runClimberCommand())

            .onFalse(m_ClimberSubsystem.stopClimberCommand());

       

        m_joystickDriverController.button(8)

            .onTrue(m_ClimberSubsystem.runClimberReverseCommand())

            .onFalse(m_ClimberSubsystem.stopClimberCommand());

    }

        // --- limelight driver ---

     

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

