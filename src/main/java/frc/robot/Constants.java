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

        // Puertos USB del control
        public static final int kJoystickPort = 0;
        public static final int kPlayJoystick_Cool_Port = 2;
        
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
        public static final double kDistanceTolerance = 1.0; 
        
        public static final double kTargetTy = 0.0; 

        // --- Tag IDs ---
        public static final int kRedHubTag1 = 9;
        public static final int kRedHubTag2 = 10;

        public static final int kBlueHubTag1 = 25;
        public static final int kBlueHubTag2 = 26;

        public static final int kRedTrenchTagAllianceTOP_left = 7;
        public static final int kRedTrenchTagCenterTOP_left = 6;
        
        public static final int kBlueTrenchTagAllianceTOP_right = 28;
        public static final int kBlueTrenchTagCenterTOP_right = 17;

        public static final int kRedTrenchTagAllianceBOTTOM_right    = 12;
        public static final int kRedTrenchTagCenterBOTTOM_right = 1;
        
        public static final int kBlueTrenchTagAllianceBOTTOM_left = 22;
        public static final int kBlueTrenchTagCenterBOTTOM_left = 23;

    }
    
    public static class TunableConstants {
        public static double shooterSpeed = 1.0; 
        public static double intakeSpeed = 1.0;
        public static double intakeReverseSpeed = -0.5;
        public static double beltIndexerSpeed = 1.0;
        public static double feederSpeed = 1.0;
        public static double intakeElevarSpeedReverse = 0.2;
        public static double intakeElevarSpeed = -0.25;

        
        public static double shooterRPMTolerance = 100.0;

        public static void publishDefaults() {
            SmartDashboard.putNumber("Tune/ShooterSpeed", shooterSpeed);
            SmartDashboard.putNumber("Tune/IntakeSpeed", intakeSpeed);
            SmartDashboard.putNumber("Tune/ShooterRPMTolerance", shooterRPMTolerance);
            
            SmartDashboard.putNumber("Tune/AlignP", VisionConstants.kAlignP);
            SmartDashboard.putNumber("Tune/AlignD", VisionConstants.kAlignD);
        }

        public static void updateFromDashboard() {
            shooterSpeed = SmartDashboard.getNumber("Tune/ShooterSpeed", shooterSpeed);
            intakeSpeed = SmartDashboard.getNumber("Tune/IntakeSpeed", intakeSpeed);
            shooterRPMTolerance = SmartDashboard.getNumber("Tune/ShooterRPMTolerance", shooterRPMTolerance);
        }
    }
}