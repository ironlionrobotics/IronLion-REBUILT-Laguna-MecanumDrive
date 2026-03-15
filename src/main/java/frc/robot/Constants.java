package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public final class Constants {
    public static final Timer timer = new Timer(); 
    
    public static class DriveConstants {
        // Puertos CAN de los motores
        public static final int kFrontLeftMotorPort = 4;
        public static final int kFrontRightMotorPort = 2;
        public static final int kRearLeftMotorPort = 1;
        public static final int kRearRightMotorPort = 3;
        public static final int kGyroPort = 30; 
        // public static final int kNeoIntakePort = 6;
        
        // Puerto USB del control
        public static final int kJoystickPort = 0;
        public static final int kJoystick_Cool_Port = 2;
        
        
        // Circunferencia de la rueda en metros
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // 4 pulgadas = 0.1016 metros 
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double gearRatio = 5.95; 
        public static final double conversionFactor = kWheelCircumferenceMeters / gearRatio;
        
        // voltage constants
        public static final double SHOOTER_SPEED = .5; // 4 pulgadas = 0.1016 metros 
        public static final double BELT_INDEXER_SPEED = 1;
        public static final double FEEDER_SPEED = 1;

    }
    public static final class AutoConstants {
        
    }
}
