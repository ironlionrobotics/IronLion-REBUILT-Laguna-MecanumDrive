package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * La clase Constants centraliza TODA la configuración del robot.
 * Regla de oro: Si un número puede cambiar en competencia, debe estar aquí.
 */
public final class Constants {
    
    // =====================================================================
    // CONSTANTES TUNABLES EN VIVO (Sin Re-Deploy)
    // Estas se pueden modificar desde SmartDashboard/Shuffleboard mientras
    // el robot está encendido. Ideal para ajustes rápidos en el pit.
    // =====================================================================
    public static class TunableConstants {
        // --- Velocidades ajustables ---
        public static double shooterSpeed = 0.5;
        public static double intakeSpeed = 0.5;
        public static double intakeReverseSpeed = -1.0;
        public static double beltIndexerSpeed = 1.0;
        public static double feederSpeed = 1.0;
        
        // --- Velocidad del chasis ---
        public static double driveSpeedMultiplier = 3.0;
        public static double turboMultiplier = 4.5;
        public static double precisionMultiplier = 1.2;
        
        // --- PID de Alineación Visual ---
        public static double alignP = 0.05;
        public static double alignI = 0.0;
        public static double alignD = 0.005;
        public static double alignTolerance = 0.5;
        
        // --- Tolerancia de RPM del Shooter ---
        public static double shooterRPMTolerance = 100.0;

        /**
         * Publica todos los valores tunables al SmartDashboard.
         * Llamar UNA VEZ en robotInit().
         */
        public static void publishDefaults() {
            SmartDashboard.putNumber("Tune/ShooterSpeed", shooterSpeed);
            SmartDashboard.putNumber("Tune/IntakeSpeed", intakeSpeed);
            SmartDashboard.putNumber("Tune/DriveSpeed", driveSpeedMultiplier);
            SmartDashboard.putNumber("Tune/TurboSpeed", turboMultiplier);
            SmartDashboard.putNumber("Tune/PrecisionSpeed", precisionMultiplier);
            SmartDashboard.putNumber("Tune/AlignP", alignP);
            SmartDashboard.putNumber("Tune/AlignD", alignD);
            SmartDashboard.putNumber("Tune/AlignTolerance", alignTolerance);
            SmartDashboard.putNumber("Tune/ShooterRPMTolerance", shooterRPMTolerance);
        }

        /**
         * Lee los valores actualizados desde SmartDashboard.
         * Llamar en periodic() del DashboardSubsystem.
         */
        public static void updateFromDashboard() {
            shooterSpeed = SmartDashboard.getNumber("Tune/ShooterSpeed", shooterSpeed);
            intakeSpeed = SmartDashboard.getNumber("Tune/IntakeSpeed", intakeSpeed);
            driveSpeedMultiplier = SmartDashboard.getNumber("Tune/DriveSpeed", driveSpeedMultiplier);
            turboMultiplier = SmartDashboard.getNumber("Tune/TurboSpeed", turboMultiplier);
            precisionMultiplier = SmartDashboard.getNumber("Tune/PrecisionSpeed", precisionMultiplier);
            alignP = SmartDashboard.getNumber("Tune/AlignP", alignP);
            alignD = SmartDashboard.getNumber("Tune/AlignD", alignD);
            alignTolerance = SmartDashboard.getNumber("Tune/AlignTolerance", alignTolerance);
            shooterRPMTolerance = SmartDashboard.getNumber("Tune/ShooterRPMTolerance", shooterRPMTolerance);
        }
    }

    // =====================================================================
    // CONSTANTES DEL CHASIS MECANUM
    // =====================================================================
    public static class DriveConstants {
        // --- Puertos CAN ---
        public static final int kFrontLeftMotorPort = 4;
        public static final int kFrontRightMotorPort = 2;
        public static final int kRearLeftMotorPort = 1;
        public static final int kRearRightMotorPort = 3;
        public static final int kGyroPort = 30; 
        
        // --- Puertos Joystick ---
        public static final int kJoystickPort = 0;
        public static final int kJoystick_Cool_Port = 2;
        public static final double kJoystickDeadband = 0.1;
        
        // --- Matemáticas Mecánicas ---
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double gearRatio = 5.95; 
        public static final double conversionFactor = kWheelCircumferenceMeters / gearRatio;
        
        // --- Geometría del Chasis (metros desde el centro) ---
        public static final Translation2d m_frontLeftLocation = new Translation2d(0.31, 0.21);
        public static final Translation2d m_frontRightLocation = new Translation2d(0.31, -0.21);
        public static final Translation2d m_backLeftLocation = new Translation2d(-0.31, 0.21);
        public static final Translation2d m_backRightLocation = new Translation2d(-0.31, -0.21);
        
        // --- Protección Eléctrica ---
        public static final int kDriveCurrentLimit = 40; 

        // --- FeedForward del Chasis (Obtener de SysId) ---
        // kS = Voltaje mínimo para vencer la fricción estática
        // kV = Voltaje por m/s de velocidad
        // kA = Voltaje por m/s² de aceleración
        public static final double kS = 1.0;
        public static final double kV = 3.0;
        public static final double kA = 4.0;
        
        // --- Velocidad Máxima para desaturación Mecanum ---
        public static final double kMaxDesaturationSpeed = 0.5;

        // --- PID de PathPlanner (AutoBuilder) ---
        public static final double kPP_TranslationP = 2.0;
        public static final double kPP_TranslationI = 0.0;
        public static final double kPP_TranslationD = 0.0;
        public static final double kPP_RotationP = 2.0;
        public static final double kPP_RotationI = 0.0;
        public static final double kPP_RotationD = 0.0;
    }

    // =====================================================================
    // CONSTANTES DEL INTAKE
    // =====================================================================
    public static class IntakeConstants {
        public static final int kNeoIntakePort = 8;
        public static final int kIntakeElevarPort = 5;

        public static final double kIntakeSpeed = 0.5;
        public static final double kIntakeReverseSpeed = -1.0;
        public static final double kElevatorSpeed = 0.1;

        public static final int kIntakeCurrentLimit = 20; 
        public static final int kElevatorCurrentLimit = 20;
    }

    // =====================================================================
    // CONSTANTES DEL SHOOTER
    // =====================================================================
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

        // --- Tabla de Calibración: Distancia (ty) → RPM ---
        // INSTRUCCIONES: Añadir filas después de cada sesión de práctica.
        // Formato: {valor_ty_limelight, RPM_deseadas}
        public static final double[][] kShooterCalibration = {
            {-10.0, 1500.0},  // Muy lejos
            {-5.0,  2250.0},  // Lejos
            {0.0,   3000.0},  // Medio
            {5.0,   3750.0},  // Cerca
            {10.0,  4500.0},  // Muy cerca
        };

        // Tolerancia para considerar que el shooter alcanzó la velocidad deseada
        public static final double kRPMTolerance = 100.0;
    }

    // =====================================================================
    // CONSTANTES DE VISIÓN (Limelight)
    // =====================================================================
    public static class VisionConstants {
        public static final String kFrontCameraName = "limelight-front";
        public static final String kBackCameraName = "limelight-back";

        // PID de Auto-Alineación (valores por defecto, se pueden tunear en vivo)
        public static final double kAlignP = 0.05;
        public static final double kAlignI = 0.0;
        public static final double kAlignD = 0.005;
        public static final double kAlignTolerance = 0.5;
    }

    // =====================================================================
    // CONSTANTES DE LEDs
    // =====================================================================
    public static class LEDConstants {
        public static final int kPWMPort = 0;
        public static final int kLedLength = 30;
    }

    // =====================================================================
    // CONSTANTES DEL CAMPO
    // =====================================================================
    public static class FieldConstants {
        public static final edu.wpi.first.math.geometry.Pose2d m_blueHubPose = 
            new edu.wpi.first.math.geometry.Pose2d(1.5, 5.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
            
        public static final edu.wpi.first.math.geometry.Pose2d m_redHubPose = 
            new edu.wpi.first.math.geometry.Pose2d(15.0, 5.5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
    }

    public static final class AutoConstants {
        // Reservado para constantes globales de PathPlanner
    }
}
