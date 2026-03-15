// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DataLogManager;
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
        motorConfig();  

    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        resetEncoders();
        SignalLogger.setPath("/u/logs");

        DataLogManager.start();
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
        ShooterSubsystem.m_NEOshooter.set(SmartDashboard.getNumber("shooter voltage", Constants.DriveConstants.SHOOTER_SPEED));
       } else if (DriveSubsystem.m_joystick.getAButtonReleased()){
        ShooterSubsystem.m_NEOshooter.set(0);
       } else if (DriveSubsystem.m_joystick.getLeftBumperButtonPressed()) {
        ShooterSubsystem.m_NEObeltIndexer.set(SmartDashboard.getNumber("belt indexer voltage", Constants.DriveConstants.BELT_INDEXER_SPEED));
        ShooterSubsystem.m_NEOfeeder.set(SmartDashboard.getNumber("feeder voltage", Constants.DriveConstants.FEEDER_SPEED));
       } else if (DriveSubsystem.m_joystick.getLeftBumperButtonReleased()) {
        ShooterSubsystem.m_NEObeltIndexer.set(0);
        ShooterSubsystem.m_NEOfeeder.set(0);
       } else if (DriveSubsystem.m_joystick.getXButtonPressed()) {
        IntakeSubsystem.m_NeoIntake.set(-1);
       } else if (DriveSubsystem.m_joystick.getXButtonReleased()) {
        IntakeSubsystem.m_NeoIntake.set(0);
       } else if (DriveSubsystem.m_joystick.getLeftTriggerAxis() == 1) {
        IntakeSubsystem.m_IntakeElevar.set(.1);
       } else if (DriveSubsystem.m_joystick.getRightTriggerAxis() == 1) {
        IntakeSubsystem.m_IntakeElevar.set(-.1);
       } else {
        IntakeSubsystem.m_IntakeElevar.set(0);
       }
             
        // movimiento field-oriented
        //TODO: change to sysID

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
        DriveSubsystem.m_gyro.setYaw(0);
        DriveSubsystem.m_gyro.reset();
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