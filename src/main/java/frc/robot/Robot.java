// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * Constructor del Robot. Se llama una sola vez al iniciar el programa.
     */
    public Robot() {
        // Configurar los factores de conversion de los encoders
        motorConfig();  
        // Configurar el chasis Mecanum

    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    @Override
    public void robotInit() {
        resetEncoders();
        
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());


        //TODO:add smart dashboard values for trajectory and pose
        
    }
    
    /**
     * Esta funcion se llama una vez al iniciar el modo autonomo.
     */
    @Override
    public void autonomousInit() {
        // Resetear todo al empezar el autonomo para una medicion limpia
        resetEncoders();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
;
        }
        // Limpiar datos anteriores del SmartDashboard

        Constants.timer.restart();
        Constants.timer.start();


    }

    /**
     * Esta funcion se llama periodicamente (cada ~20ms) durante el modo autonomo.
     */
    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("X Error", DriveSubsystem.m_xController.getError());
        SmartDashboard.putNumber("Y Error", DriveSubsystem.m_yController.getError());

    }

    @Override
    public void teleopInit() {
        // 4. CANCELAR AUTO AL ENTRAR A TELEOP
        // Esto evita que el robot siga intentando hacer la ruta si tomas el control
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * Esta funcion se llama periodicamente durante el modo teleoperado.
     */
    @Override
    public void teleopPeriodic() {
        if (DriveSubsystem.m_joystick.getAButtonPressed()) {
            ShooterSubsystem.m_NEOshooter.set(1);
        } else if (DriveSubsystem.m_joystick.getAButtonReleased()){
            ShooterSubsystem.m_NEOshooter.set(0);
        } else if (DriveSubsystem.m_joystick.getLeftBumperButtonPressed()) {
            ShooterSubsystem.m_NEObeltIndexer.set(1);
            ShooterSubsystem.m_NEOfeeder.set(1);
        } else if (DriveSubsystem.m_joystick.getLeftBumperButtonReleased()) {
            ShooterSubsystem.m_NEObeltIndexer.set(0);
            ShooterSubsystem.m_NEOfeeder.set(0);
        }
    //              
    //              double avanzar = -DriveSubsystem.m_joystick.getLeftY(); 
    //              // 
    //              double lateral = DriveSubsystem.m_joystick.getLeftX(); 
    //              // 
    //              double rotar = DriveSubsystem.m_joystick.getRightX(); 
//          
    //              // Aplicamos una deadzone simple para que no zumbe el motor si el gatillo no regresa a cero exacto
//          
    //              if (DriveSubsystem.m_joystick.getAButton()) {
    //                  IntakeSubsystem.m_NeoIntake.set(-0.6);
    //              } else {
    //                  IntakeSubsystem.m_NeoIntake.set(0);
    //              }
    //              // Control de Intake (Lanzar/Succionar)
    //              if (DriveSubsystem.m_joystick.getLeftTriggerAxis() > .7) {
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(.7); 
//          
    //              } else if (DriveSubsystem.m_joystick.getLeftTriggerAxis() > .5) {
    //                  
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(0.4);
    //              } else if (DriveSubsystem.m_joystick.getLeftTriggerAxis() > .3) {
    //                  
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(0.15);
    //              } 
    //              // Control de Outtake (Invertido)
    //              else if (DriveSubsystem.m_joystick.getRightTriggerAxis() > .7) {
    //                  
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(-.7);
    //              } else if (DriveSubsystem.m_joystick.getRightTriggerAxis() > .5) {
    //                  
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(-0.4);
    //              } else if (DriveSubsystem.m_joystick.getRightTriggerAxis() > .2) {
    //                  
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(-0.15);
    //              } 
    //              // Si no se presiona nada, detener el motor
    //              else {
    //                  IntakeSubsystem.m_IntakeElevar.setVoltage(0.40);
    //              }
//          
    //              // Esto hace el movimiento field-oriented
    //              DriveSubsystem.m_robotDrive.driveCartesian(
    //                  avanzar * 0.2,  // Forward/Backwards 
    //                  lateral * 0.2,  // Strafe
    //                  rotar * 0.3, // Rotacion
    //                  DriveSubsystem.m_gyro.getRotation2d().unaryMinus()); // unaryMinus porque funciona xddddd  
    //                  
    //              DriveSubsystem.updateOdometry();
    }

    // --- FUNCIONES AUXILIARES ---
    /**
     * Reinicia la posicion de los encoders de los motores y giroscopio a 0.
     */
    public void resetEncoders() {
        DriveSubsystem.m_frontLeft.getEncoder().setPosition(0);
        DriveSubsystem.m_rearLeft.getEncoder().setPosition(0);
        DriveSubsystem.m_frontRight.getEncoder().setPosition(0);
        DriveSubsystem.m_rearRight.getEncoder().setPosition(0);
        DriveSubsystem.m_gyro.reset();
        DriveSubsystem.m_gyro.setYaw(0);
    }
    
    public void motorConfig() {
                SparkMaxConfig commmConfig = new SparkMaxConfig();
        commmConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        commmConfig.encoder
            .positionConversionFactor(DriveConstants.conversionFactor)
            .velocityConversionFactor(DriveConstants.conversionFactor / 60.0); // conversion a metros por segundo
        // Invertir los motores del lado izquierdo.        
        SparkBaseConfig leftConfig = new SparkMaxConfig().idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftConfig.apply(commmConfig);
        leftConfig.inverted(false);
        
        SparkBaseConfig rightConfig = new SparkMaxConfig().idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightConfig.apply(commmConfig);
        rightConfig.inverted(true);

        SparkBaseConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeConfig.inverted(true);

        SparkBaseConfig intakeElevarConfig = new SparkMaxConfig();
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeConfig.inverted(false);

        // El lado que se invierte depende del cableado y mecanica del robot.
        DriveSubsystem.m_frontLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        DriveSubsystem.m_rearLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DriveSubsystem.m_frontRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        DriveSubsystem.m_rearRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IntakeSubsystem.m_NeoIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IntakeSubsystem.m_IntakeElevar.configure(intakeElevarConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
       
}