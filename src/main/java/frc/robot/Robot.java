// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;

/**
 * La clase Robot es el "cerebro" principal que controla el ciclo de vida del robot.
 * Hereda de TimedRobot, lo que significa que sus métodos se ejecutan periódicamente
 * (por defecto cada 20 milisegundos).
 */
public class Robot extends TimedRobot {

    // Comando que se ejecutará durante el periodo autónomo
    private Command m_autonomousCommand;
    // Contenedor principal donde se definen subsistemas y controles (la "columna vertebral")
    private RobotContainer m_robotContainer;

    public Robot() {
    }

    /**
     * Este método se ejecuta constantemente cada 20ms sin importar el modo del robot
     * (Autónomo, Teleoperado, Deshabilitado). 
     */
    @Override
    public void robotPeriodic() {
        // Ejecuta el Scheduler de Comandos. Esto es lo que hace que los "Commands" y "Subsystems" funcionen.
        // Revisa qué comandos están activos y llama a sus métodos asociados. ¡Es OBLIGATORIO que esto se corra!
        CommandScheduler.getInstance().run();
    }

    /**
     * Este método se ejecuta UNA SOLA VEZ cuando el robot se enciende o se reinicia su código.
     * Es idea para crear los objetos e inicializar sensores.
     */
    @Override
    public void robotInit() {
        // Instanciamos el RobotContainer. Es aquí donde configuramos controles y módulos.
        m_robotContainer = new RobotContainer();
        
        // Configuramos la ruta para los logs de hardware (telemetría de motores Phoenix)
        SignalLogger.setPath("/u/logs");

        // Esta utilidad nativa graba variales y telemetría a un pendrive USB automáticamente, muy útil para diagnosticar errores.
        DataLogManager.start();
        
        // Pre-carga las rutas de PathPlanner en RAM. Esto ayuda a que el robot no se quede trabado medio segundo el iniciar el modo Autónomo.
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }
    
    /**
     * Este método se llama UNA SOLA VEZ cuando inicia el periodo Autónomo Oficial (primeros 20 seg).
     */
    @Override
    public void autonomousInit() {
        // Obtenemos del RobotContainer el comando autónomo que el conductor pidió desde la Pantalla (Dashboard)
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        // Si sí elegimos un comando, lo metemos al Scheduler para que empiece a correr
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        // Reinicia e inicia nuestro propio temporizador (para la lógica propia de tiempo)
        Constants.timer.restart();
        Constants.timer.start();
    }

    /**
     * Este método se ejecuta periódicamente (cada 20ms) DURANTE todo el periodo autónomo.
     */
    @Override
    public void autonomousPeriodic() {
        // Se deja vacío comúnmente ya que la arquitectura basada en comandos maneja el control automáticamente en el `robotPeriodic`.
    }       

    /**
     * Este método se llama UNA SOLA VEZ cuando termina el autónomo e inicia el Teleoperado (humanos al volante).
     */
    @Override
    public void teleopInit() {
        // Cancelamos fuertemente el comando autónomo si este seguía corriendo (por si la rutina no terminó a tiempo).
        // Así los conductores recuperan el control inmediatamente.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * Este método se ejecuta periódicamente (cada 20ms) DURANTE el modo Teleoperado.
     */
    @Override
    public void teleopPeriodic() {
        // Vacío, el control de comandos funciona leyendo los joysticks (ver RobotContainer).
    }

}