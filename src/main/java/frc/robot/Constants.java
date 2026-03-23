package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {
    
    public static class DriveConstants {
        // Puertos CAN de los motores
        public static final int kFrontLeftMotorPort = 4;
        public static final int kFrontRightMotorPort = 2;
        public static final int kRearLeftMotorPort = 1;
        public static final int kRearRightMotorPort = 3;
        public static final int kGyroPort = 30; 
        
        public static final int kNeoIntakePort = 8;
        public static final int kIntakeArmPort = 5;
        public static final int kNeoBeltIndexerPort = 9;
        public static final int kNeoFeederPort = 10;
        public static final int kNeoShooterPort = 11; 
        public static final int kNeoClimberPort = 7; 

        // Puerto USB del control
        public static final int kJoystickPort = 0;
        public static final int kPlayJoystick_Cool_Port = 1;
        public static final int kJoystick_Cool_Port = 2;
        
        // Matemáticas Físicas
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        
        public static final double gearRatio = 5.95; 
        public static final double conversionFactor = kWheelCircumferenceMeters / gearRatio;
        
        // --- SysId / FeedForward Realistas (NEO + Mecanum) ---
        public static final double kS = 0.2; 
        public static final double kV = 2.8; 
        public static final double kA = 0.3; 
    }

    public static final class VisionConstants {
        // --- Rotation (tx) Constants ---
        public static final double kAlignP = 0.015; 
        public static final double kAlignI = 0.0;
        public static final double kAlignD = 0.001; 
        public static final double kAlignTolerance = 1.0; 
        
        // --- Distance (ty) Constants ---
        public static final double kDistanceP = 0.1; 
        public static final double kDistanceI = 0.0;
        public static final double kDistanceD = 0.0;
        public static final double kDistanceTolerance = 1.0; // degrees of ty error allowed
        
        public static final double kTargetTy = 0.0; // negative = farther, positve = nearer
    }
    
    public static class TunableConstants {
        public static double shooterSpeed = 1; 
        public static double intakeSpeed = 1;
        public static double intakeReverseSpeed = -0.5;
        public static double beltIndexerSpeed = 1.0;
        public static double feederSpeed = 1.0;
        public static double intakeElevarSpeedReverse = 0.2;
        public static double intakeElevarSpeed = -1;

        public static double driveSpeedMultiplier = 3.0; 
        public static double turboMultiplier = 4.5;
        public static double precisionMultiplier = 1.2;
        
        public static double shooterRPMTolerance = 100.0;
        public static double manualTargetRPM = 3000.0;

        public static void publishDefaults() {
            SmartDashboard.setDefaultNumber("Tune/ShooterSpeed", shooterSpeed);
            SmartDashboard.setDefaultNumber("Tune/ManualTargetRPM", manualTargetRPM);
            SmartDashboard.setDefaultNumber("Tune/IntakeSpeed", intakeSpeed);
            SmartDashboard.setDefaultNumber("Tune/DriveSpeed", driveSpeedMultiplier);
            SmartDashboard.setDefaultNumber("Tune/TurboSpeed", turboMultiplier);
            SmartDashboard.setDefaultNumber("Tune/PrecisionSpeed", precisionMultiplier);
            SmartDashboard.setDefaultNumber("Tune/ShooterRPMTolerance", shooterRPMTolerance);
            
            SmartDashboard.putNumber("Tune/AlignP", VisionConstants.kAlignP);
            SmartDashboard.putNumber("Tune/AlignD", VisionConstants.kAlignD);
        }

        public static void updateFromDashboard() {
            shooterSpeed = SmartDashboard.getNumber("Tune/ShooterSpeed", shooterSpeed);
            manualTargetRPM = SmartDashboard.getNumber("Tune/ManualTargetRPM", manualTargetRPM);
            intakeSpeed = SmartDashboard.getNumber("Tune/IntakeSpeed", intakeSpeed);
            driveSpeedMultiplier = SmartDashboard.getNumber("Tune/DriveSpeed", driveSpeedMultiplier);
            turboMultiplier = SmartDashboard.getNumber("Tune/TurboSpeed", turboMultiplier);
            precisionMultiplier = SmartDashboard.getNumber("Tune/PrecisionSpeed", precisionMultiplier);
            shooterRPMTolerance = SmartDashboard.getNumber("Tune/ShooterRPMTolerance", shooterRPMTolerance);
        }
    }
}