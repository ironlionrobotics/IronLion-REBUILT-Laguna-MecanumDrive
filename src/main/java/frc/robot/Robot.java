// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;
import frc.robot.Subsystems.DriveSubsystem;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * Constructor del Robot. Se llama una sola vez al iniciar el programa.
     */
    public Robot() {
        DriveSubsystem.motorConfig();  

    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
        resetEncoders();
        //TODO: SET path for signal logger
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


    }

    /**
     * Esta funcion se llama periodicamente (cada ~20ms) durante el modo autonomo.
     */
    @Override
    public void autonomousPeriodic() {

    }       

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * Esta funcion se llama periodicamente durante el modo teleoperado.
     */
    @Override
    public void teleopPeriodic() {

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
       
}