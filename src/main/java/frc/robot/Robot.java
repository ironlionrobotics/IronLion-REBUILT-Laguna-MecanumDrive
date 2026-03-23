package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.DriveSubsystem;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    public Robot() {
        DriveSubsystem.motorConfig();  
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Constants.TunableConstants.updateFromDashboard();
    }
    
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture(); // Starts USB Camera
        resetEncoders();
        
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }
    
    @Override
    public void autonomousInit() {
        resetEncoders();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}       

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    public void resetEncoders() {
        DriveSubsystem.m_frontLeft.getEncoder().setPosition(0);
        DriveSubsystem.m_rearLeft.getEncoder().setPosition(0);
        DriveSubsystem.m_frontRight.getEncoder().setPosition(0);
        DriveSubsystem.m_rearRight.getEncoder().setPosition(0);
        DriveSubsystem.m_gyro.setYaw(0);
        DriveSubsystem.m_gyro.reset();
    }       
}