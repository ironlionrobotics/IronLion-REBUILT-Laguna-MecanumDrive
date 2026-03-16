package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

/**
 * DriveSubsystem es el corazón del movimiento del robot. 
 * Maneja todo el chasis Mecanum, incluyendo motores, odometría (saber en qué X/Y del campo estamos),
 * giroscopio y la interfaz con el piloto automático autónomo de PathPlanner.
 */
public class DriveSubsystem extends SubsystemBase  {
  
    // ---------------------------------------------------------
    // HARDWARE
    // ---------------------------------------------------------
    // Motores principales del chasis (Controladores tipo SparkMax Brushless).
    private final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_rearLeft = new SparkMax(DriveConstants.kRearLeftMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_rearRight = new SparkMax(DriveConstants.kRearRightMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    // El sensor giroscópico Pigeon2 (Proporciona el Yaw o Brújula)
    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroPort);
    
    // Controladores PID para la velocidad individual de cada llanta.
    private final PIDController m_frontLeftPIDController = new PIDController(DriveConstants.kP_X, 0, 0);
    private final PIDController m_frontRightPIDController = new PIDController(DriveConstants.kP_X, 0, 0);
    private final PIDController m_backLeftPIDController = new PIDController(DriveConstants.kP_X, 0, 0);
    private final PIDController m_backRightPIDController = new PIDController(DriveConstants.kP_X, 0, 0);

    // FeedForward (Fuerza Predictiva): Ayuda enviando fuerza base tomando en cuenta resistencia/peso físico(fricción).
    // Valores centralizados en Constants.DriveConstants (kS, kV, kA) para fácil tuneo.
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    // ---------------------------------------------------------
    // CINEMÁTICA Y NAVEGACIÓN (HOLONOMIC DRIVE)
    // ---------------------------------------------------------
    // Controladores PIDER globales del robot entero para moverse por el campo hacia coordenadas específicas.
    private final PIDController m_xController = new PIDController(DriveConstants.kP_X, DriveConstants.kI_X, DriveConstants.kD_X);
    private final PIDController m_yController = new PIDController(DriveConstants.kP_Y, DriveConstants.kI_Y, DriveConstants.kD_Y);

    // Este controlador domina cómo viaja el robot en modo manual asistido calculando la cinemática global.
    private final HolonomicDriveController m_driveController = 
      new HolonomicDriveController(
        m_xController,
        m_yController,
        // Theta = Rotación. TrapezoidProfile hace que la rotación arranque y frene suave en forma de "campana".
        new ProfiledPIDController(DriveConstants.kP_Theta, DriveConstants.kI_Theta, DriveConstants.kD_Theta,
        new TrapezoidProfile.Constraints(6.28, 3.14)));
        
    // Cinemática Mecanum: Realiza matemáticas complejas sabiendo dónde está cada rueda para lograr strafe(caminar de cangrejo)
    private final MecanumDriveKinematics m_kinematics = 
      new MecanumDriveKinematics(
        DriveConstants.m_frontLeftLocation, DriveConstants.m_frontRightLocation, DriveConstants.m_backLeftLocation, DriveConstants.m_backRightLocation
      );
    
    // Objeto primitivo que trackeaba dónde estamos leyendo solo las llantas.
    private final MecanumDriveOdometry m_odometry;

    // EL Estimador "Mega" usado para mezclar Odometría + Visión(Cámara). Si las llantas patinan, la cámara corrige el GPS.
    private final MecanumDrivePoseEstimator m_poseEstimator;

    /**
     * Constructor llamado por el RobotContainer cuando el robot inicia.
     */
    public DriveSubsystem() {
        // --- CONFIGURACIÓN ELÉCTRICA DE MOTORES ---
        com.revrobotics.spark.config.SparkMaxConfig commmConfig = new com.revrobotics.spark.config.SparkMaxConfig();
        // Mode Brake hace que frene seco al dejar el control, Coast deja que ruede. CurrentLimit aplica el tope de fusibles a amperios.
        commmConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                   .smartCurrentLimit(DriveConstants.kDriveCurrentLimit);
        // Indicamos directamente a los controladores cómo convertir lecturas RPM del motor a un valor en Metros/Segundos
        commmConfig.encoder
            .positionConversionFactor(DriveConstants.conversionFactor)
            .velocityConversionFactor(DriveConstants.conversionFactor / 60.0);
        
        // Invertimos el motor derecho porque en ensambles de chasis "apuntan" al reves. 
        com.revrobotics.spark.config.SparkBaseConfig leftConfig = new com.revrobotics.spark.config.SparkMaxConfig().idleMode(com.revrobotics.spark.config.SparkMaxConfig.IdleMode.kBrake);
        leftConfig.apply(commmConfig);
        leftConfig.inverted(false);
        
        com.revrobotics.spark.config.SparkBaseConfig rightConfig = new com.revrobotics.spark.config.SparkMaxConfig().idleMode(com.revrobotics.spark.config.SparkMaxConfig.IdleMode.kBrake);
        rightConfig.apply(commmConfig);
        rightConfig.inverted(true); // ¡Punto clave! Los motores derechos van inversos.

        // Mandamos inyectar la configuración al hardware FÍSICO guardando en la EPROM.
        m_frontLeft.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        m_rearLeft.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        m_frontRight.configure(rightConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        m_rearRight.configure(rightConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

        // Inicializar la Odometría con los encoders reseteados a 0.
        m_odometry = new MecanumDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            getCurrentDistances(),
            Pose2d.kZero
        );

        // Inicializamos el estimador de GPS robótico (Fusión de Odometría de Ruedas + Visión).
        m_poseEstimator = new MecanumDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          getCurrentDistances(),
          Pose2d.kZero,
          // Constantes de desviación Estándard. Qué confiar más, si en LLANTAS o en la CÁMARA (Limelight).
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // Confiabilidad codificadores
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))   // Confiabilidad visión
        );
    }

    /** Resetear de cero el sistema a una coordenada solicitada. (Acomodar al inicio del Match) */
    public void resetOdometry(Pose2d pose) {
        m_gyro.reset();
        m_frontLeft.getEncoder().setPosition(0);
        m_rearLeft.getEncoder().setPosition(0);
        m_frontRight.getEncoder().setPosition(0);
        m_rearRight.getEncoder().setPosition(0);
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getCurrentDistances(), pose);
    }
    
    /** Periódico interno (Cada 20ms actualizas la coordenada de GPS) */
    public void updateOdometry() {
        m_poseEstimator.update(m_gyro.getRotation2d().unaryMinus(), getCurrentDistances());
    }

    // --- INTEGRACIÓN VISION (Capa Fase 4) ---
    /** Recibe la posición calculada por los Tags April desde la Limelight y corrige el GPS del robot. */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    // Retorna la Pose Actual (X,Y, Ángulo) en Metros. Equivalente a la coordenada Satelital.
    public Pose2d getPose() {
      return m_poseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }

    /** Pone la intención de manejar del control a velocidades físicas de todas las ruedas */
    public void driveRobotRelative(ChassisSpeeds speeds) {
      MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
      wheelSpeeds.desaturate(DriveConstants.kMaxDesaturationSpeed);  // Velocidad máxima centralizada en Constants
      setSpeeds(wheelSpeeds);
    }
    
    /** Cuánta distancia individual (en metros) ha recorrido cada llanta */
    public MecanumDriveWheelPositions getCurrentDistances() {
        return new MecanumDriveWheelPositions( 
            m_frontLeft.getEncoder().getPosition()  * DriveConstants.conversionFactor,
            m_rearLeft.getEncoder().getPosition()   * DriveConstants.conversionFactor,
            m_frontRight.getEncoder().getPosition() * DriveConstants.conversionFactor,
            m_rearRight.getEncoder().getPosition()  * DriveConstants.conversionFactor
          );
    }

    /** Regresa la velocidad cruda por llanta en RPS y se adapta a Metros x Segundo (MPS) */
    public MecanumDriveWheelSpeeds getCurrentState() { 
            double frontLeftRPS = m_frontLeft.getEncoder().getVelocity()   / 60.0;
            double frontRightRPS = m_frontRight.getEncoder().getVelocity() / 60.0;
            double rearLeftRPS = m_rearLeft.getEncoder().getVelocity()     / 60.0;
            double rearRightRPS = m_rearRight.getEncoder().getVelocity()   / 60.0;

      return new MecanumDriveWheelSpeeds(
        frontLeftRPS * DriveConstants.conversionFactor,
        rearLeftRPS * DriveConstants.conversionFactor,
        frontRightRPS * DriveConstants.conversionFactor,
        rearRightRPS * DriveConstants.conversionFactor
        );
    }

    /** Establecer Velocidad Individual y convertida desde un calculador PID Interno hacia Voltaje Neto (V) */
    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        double velocityConversionFactor = 60.0 / DriveConstants.conversionFactor;

        // Lee donde estamos
        final double frontLeftCurrent = m_frontLeft.getEncoder().getVelocity() / velocityConversionFactor;
        final double frontRightCurrent = m_frontRight.getEncoder().getVelocity() / velocityConversionFactor; 
        final double rearLeftCurrent = m_rearLeft.getEncoder().getVelocity() / velocityConversionFactor;
        final double rearRightCurrent = m_rearRight.getEncoder().getVelocity() / velocityConversionFactor;

        // Predice cuántos voltios necesita el peso de robot empujar sólo para moverse
        final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
        final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
        final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
        final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

        // Corrige el error PID si el motor se queda corto
        final double frontLeftOutput = m_frontLeftPIDController.calculate(frontLeftCurrent, speeds.frontLeftMetersPerSecond);
        final double frontRightOutput = m_frontRightPIDController.calculate(frontRightCurrent, speeds.frontRightMetersPerSecond);
        final double backLeftOutput = m_backLeftPIDController.calculate(rearLeftCurrent, speeds.rearLeftMetersPerSecond);
        final double backRightOutput = m_backRightPIDController.calculate(rearRightCurrent, speeds.rearRightMetersPerSecond);

        // Establece salida bruta combinando Corrección(PID) + Fuerza Predictiva(FeedForward)
        m_frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
        m_frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
        m_rearLeft.setVoltage(backLeftOutput + backLeftFeedforward);
        m_rearRight.setVoltage(backRightOutput + backRightFeedforward);
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds() {
      return m_kinematics.toChassisSpeeds(getCurrentState());
    }

    /**
     * Esta es la Magia del Autónomo Inteligente (Capa de Conexión a PATHPLANNER Phase 6).
     */
    public void configureAutoBuilder() {
      try {
        var config = RobotConfig.fromGUISettings();
        // Configure AutoBuilder le dice "Como Moverse"
        AutoBuilder.configure(
          this::getPose, 
          this::resetPose, 
          this::getRobotRelativeSpeeds, 
          // Acción de drive pasiva solicitada por comandos pre-planeados
          (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
              // PID de PathPlanner centralizados en Constants para tuneo rápido en competencia
              new PIDConstants(DriveConstants.kPP_TranslationP, DriveConstants.kPP_TranslationI, DriveConstants.kPP_TranslationD),
              new PIDConstants(DriveConstants.kPP_RotationP, DriveConstants.kPP_RotationI, DriveConstants.kPP_RotationD)
            ), 
            config, 
              // Lógica Auto-Flip: Revisa FMS para ver color. Si jugamos de color ROJO, invierte matemáticamente la cinemática de las curvas hacia la derecha.
              () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this 
          );
      } catch (Exception e) {
          e.printStackTrace();
      }
    }
    
  // --- RUTINA SYSID (TUNEOS MAGISTRALES TÉCNICOS) --- 
  // Crea perfiles en logfiles simulando empuje graduales para ser pasados a calculadoras python (Robotpy) o WPILib Tool.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
            double v = volts.in(edu.wpi.first.units.Units.Volts);
            m_frontLeft.setVoltage(v);
            m_frontRight.setVoltage(v);
            m_rearLeft.setVoltage(v);
            m_rearRight.setVoltage(v);
        },
        log -> {
            log.motor("drive-left")
                .voltage(Volts.of(m_frontLeft.getBusVoltage() * m_frontLeft.getAppliedOutput()))
                .linearPosition(Meters.of(m_frontLeft.getEncoder().getPosition() * DriveConstants.conversionFactor))
                .linearVelocity(MetersPerSecond.of(m_frontLeft.getEncoder().getVelocity() * (DriveConstants.conversionFactor / 60.0)));
            
            log.motor("drive-right")
                .voltage(Volts.of(m_frontRight.getBusVoltage() * m_frontRight.getAppliedOutput()))
                .linearPosition(Meters.of(m_frontRight.getEncoder().getPosition() * DriveConstants.conversionFactor))
                .linearVelocity(MetersPerSecond.of(m_frontRight.getEncoder().getVelocity() * (DriveConstants.conversionFactor / 60.0)));
        },
        this
    )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
  }

  public Pigeon2 getGyro() { return m_gyro; }
  
  /**
   * Ciclo Infinito base del hardware chasis. (Cada 20ms).
   */
  @Override
  public void periodic() {
      // 1. Siempre calcular GPS Odometría (Matemáticas sobre llantas + Giroscopio).
      updateOdometry(); 
      
      SmartDashboard.putNumber("Robot X", getPose().getX());
      SmartDashboard.putNumber("Robot Y", getPose().getY());
      SmartDashboard.putNumber("Robot Yaw", m_gyro.getRotation2d().getDegrees());

      // Telemetría para afinar PathPlanner PID y poder graficarlos en tiempo real.
      SmartDashboard.putNumber("X Error", m_xController.getPositionError());
      SmartDashboard.putNumber("Y Error", m_yController.getPositionError());
      SmartDashboard.putNumber("Theta Error", m_driveController.getThetaController().getPositionError());
  }
}