package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public final class Constants {
    
    public static class DriveConstants {
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

        public static final int kDriverControllerPort = 0; // Unified single controller
        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        
        public static final double gearRatio = 5.95; 
        public static final double conversionFactor = kWheelCircumferenceMeters / gearRatio;
        
        public static final double kS = 0.2; 
        public static final double kV = 2.8; 
        public static final double kA = 0.3; 
    }

    // --- NEW: Safety Cutoffs ---
    public static final class SafetyConstants {
        public static final double kMaxMotorTempCelsius = 60.0;
        public static final double kMaxMotorCurrentAmps = 40.0;
    }
    
    public static class TunableConstants {
        public static double shooterSpeed = 1.0; 
        public static double intakeSpeed = 1.0;
        public static double intakeReverseSpeed = -1.0;
        public static double beltIndexerSpeed = 1.0;
        public static double beltIndexerReverseSpeed = -1.0;
        public static double feederSpeed = 1.0;
        public static double feederReverseSpeed = -1.0;
        public static double intakeElevarSpeedReverse = 1.0;
        public static double intakeElevarSpeed = -0.8;
    }

    // --- NEW: Centralized Telemetry ---
    public static class Telemetry {
        public static void publishDefaults() {
            SmartDashboard.putNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            SmartDashboard.putNumber("Tune/IntakeSpeed", TunableConstants.intakeSpeed);
        }

        public static void updateConstantsFromDashboard() {
            TunableConstants.shooterSpeed = SmartDashboard.getNumber("Tune/ShooterSpeed", TunableConstants.shooterSpeed);
            TunableConstants.intakeSpeed = SmartDashboard.getNumber("Tune/IntakeSpeed", TunableConstants.intakeSpeed);
        }

        public static void updateSubsystemTelemetry(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter) {
            // Intake
            SmartDashboard.putNumber("Intake/Arm Angle", intake.getArmPosition());
            SmartDashboard.putNumber("Intake/Arm Temp (C)", intake.getArmTemp());
            SmartDashboard.putNumber("Intake/Motor Temp (C)", intake.getIntakeTemp());
            
            // Shooter
            SmartDashboard.putNumber("Shooter/Raw Power", shooter.getAppliedOutput());
            SmartDashboard.putNumber("Shooter/Temp (C)", shooter.getShooterTemp());
            SmartDashboard.putNumber("Shooter/Current (A)", shooter.getShooterCurrent());

            // Drive
            SmartDashboard.putNumber("Drive/Heading", drive.getHeading().getDegrees());
        }
    }
    public static final class VisionConstants {
        // Turning PID
        public static final double kAlignP = 0.05; 
        public static final double kAlignD = 0.001; 
        
        // Distance PID
        public static final double kDistanceP = 0.1; 
        
        // The "Distance" you want to maintain. 
        // 0.0 means keep the tag perfectly vertically centered in the camera.
        // Change to a negative number (e.g., -5.0) to stay further away.
        public static final double kTargetTy = -0.5; 
    }
}