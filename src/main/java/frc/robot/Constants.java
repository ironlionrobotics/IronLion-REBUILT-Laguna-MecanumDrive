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



        // Puerto USB del control

        public static final int kJoystickPort = 0;

        public static final int kJoystick_Cool_Port = 1;

       

       

        // Circunferencia de la rueda en metros

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // 4 pulgadas = 0.1016 metros

        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double gearRatio = 5.95;

        public static final double conversionFactor = kWheelCircumferenceMeters / gearRatio;

       

        // voltage constants



    }

    public static final class VisionConstants {

        public static final double kAlignP = 0.0;

        public static final double kAlignI = 0.0;

        public static final double kAlignD = 0.0;

        public static final double kAlignTolerance = 5.0;

    }

   

    public static class TunableConstants {

        // --- Velocidades ajustables ---

        public static double shooterSpeed = 0.4;

        public static double intakeSpeed = 1;

        public static double intakeReverseSpeed = -1.0;

        public static double beltIndexerSpeed = 1.0;

        public static double feederSpeed = 1.0;

        public static double intakeElevarSpeedReverse = 0.2;

        public static double intakeElevarSpeed = -0.25;



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

}}

