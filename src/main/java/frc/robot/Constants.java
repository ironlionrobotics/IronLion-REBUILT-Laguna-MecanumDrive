package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final Timer timer = new Timer(); 
    
    public static class DriveConstants {
        // Puertos CAN de los motores
        public static final int kFrontLeftMotorPort = 4;
        public static final int kFrontRightMotorPort = 2;
        public static final int kRearLeftMotorPort = 1;
        public static final int kRearRightMotorPort = 3;
        public static final int kGyroPort = 30; 
        
        // Puerto USB del control
        public static final int kJoystickPort = 0;
        public static final int kJoystick_Cool_Port = 2;
        public static final double kJoystickDeadband = 0.1;
        
        // Circunferencia de la rueda en metros
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double gearRatio = 5.95; 
        public static final double conversionFactor = kWheelCircumferenceMeters / gearRatio;
        
        // Geometría del chasis Mecanum
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.31, 0.21);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.31, -0.21);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.31, 0.21);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.31, -0.21);
        
        // Límites Eléctricos
        public static final int kDriveCurrentLimit = 40; // 40 Amperes por llanta Mecanum
    }

    public static class IntakeConstants {
        public static final int kNeoIntakePort = 8;
        public static final int kIntakeElevarPort = 5;

        public static final double kIntakeSpeed = 0.5;
        public static final double kIntakeReverseSpeed = -1.0;
        public static final double kElevatorSpeed = 0.1;

        public static final int kIntakeCurrentLimit = 20; // 20 Amperes
        public static final int kElevatorCurrentLimit = 20;
    }

    public static class ShooterConstants {
        public static final int kBeltIndexerPort = 9;
        public static final int kShooterPort = 11;
        public static final int kFeederPort = 10;

        public static final double kShooterSpeed = 0.5;
        public static final double kBeltIndexerSpeed = 1.0;
        public static final double kFeederSpeed = 1.0;

        public static final int kShooterCurrentLimit = 40; 
        public static final int kFeederCurrentLimit = 20;
        public static final int kBeltIndexerCurrentLimit = 20;
    }

    public static class VisionConstants {
        // Nombres que se le configurarán en el dashboard local de cada Limelight (http://limelight.local:5801)
        public static final String kFrontCameraName = "limelight-front";
        public static final String kBackCameraName = "limelight-back";

        // PID para Auto-Alineación (basado en tx)
        public static final double kAlignP = 0.05;
        public static final double kAlignI = 0.0;
        public static final double kAlignD = 0.005;
        public static final double kAlignTolerance = 0.5; // grados
    }

    public static class LEDConstants {
        public static final int kPWMPort = 0; // Puerto PWM físico en el RoboRIO
        public static final int kLedLength = 30; // 30 LEDs en la tira Neopixel
    }

    public static class FieldConstants {
        // Coordenadas hipotéticas para los HUBs de REBUILT (Blue y Red)
        // en metros, obtenidas del manual y PathPlanner
        public static final edu.wpi.first.math.geometry.Pose2d m_blueHubPose = 
            new edu.wpi.first.math.geometry.Pose2d(1.5, 5.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
            
        public static final edu.wpi.first.math.geometry.Pose2d m_redHubPose = 
            new edu.wpi.first.math.geometry.Pose2d(15.0, 5.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
    }

    public static final class AutoConstants {
        
    }
}
