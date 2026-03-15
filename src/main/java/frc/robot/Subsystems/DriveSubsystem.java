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

public class DriveSubsystem extends SubsystemBase  {
  
    public static final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_rearLeft = new SparkMax(DriveConstants.kRearLeftMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static final SparkMax m_rearRight = new SparkMax(DriveConstants.kRearRightMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    public static final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroPort);

    public static final XboxController m_joystick = new XboxController(DriveConstants.kJoystickPort);

    public static final GenericHID m_joystickCool = new GenericHID(DriveConstants.kJoystick_Cool_Port);
    
    public static final PIDController m_frontLeftPIDController = new PIDController(.2, 0, 0);
    public static final PIDController m_frontRightPIDController = new PIDController(.2, 0, 0);
    public static final PIDController m_backLeftPIDController = new PIDController(.2, 0, 0);
    public static final PIDController m_backRightPIDController = new PIDController(.2, 0, 0);

    public static final Translation2d m_frontLeftLocation = new Translation2d(0.31, 0.21);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.31, -0.21);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.31, 0.21);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.31, -0.21);
    
    //TODO: ask about these values TUNE sys id with KS, KV AND KA 
    public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3,4);

        // --- HOLONOMIC DRIVE --- //
    //TODO: tune pid controllers: x and y + profied pid (rotation), starting with .2
        public static final PIDController m_xController = new PIDController(1.0, 0, 0);
        public static final PIDController m_yController = new PIDController(1.0, 0, 0);

        public static final HolonomicDriveController m_driveController = 
          new HolonomicDriveController(
            m_xController,
            m_yController,
            new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(6.28, 3.14)));
        
    public static final MecanumDriveKinematics m_kinematics = 
      new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
      );
    
    MecanumDriveOdometry m_odometry = 
      new MecanumDriveOdometry(
        m_kinematics,
        DriveSubsystem.m_gyro.getRotation2d(),
        new MecanumDriveWheelPositions(
          m_frontLeft.getEncoder().getPosition() * DriveConstants.conversionFactor, 
          m_frontRight.getEncoder().getPosition() * DriveConstants.conversionFactor,
          m_rearLeft.getEncoder().getPosition() * DriveConstants.conversionFactor, 
          m_rearRight.getEncoder().getPosition() * DriveConstants.conversionFactor
        ),
        new Pose2d(0, 0, new Rotation2d())
      );

    public static final MecanumDrivePoseEstimator m_poseEstimator =
      new MecanumDrivePoseEstimator(
          m_kinematics,
          DriveSubsystem.m_gyro.getRotation2d(),
          getCurrentDistances(),
          Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );

    public void resetOdometry(Pose2d pose) {
        m_gyro.reset();
        m_frontLeft.getEncoder().setPosition(0);
        m_rearLeft.getEncoder().setPosition(0);
        m_frontRight.getEncoder().setPosition(0);
        m_rearRight.getEncoder().setPosition(0);
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getCurrentDistances(), pose);
    }
    
    public static void updateOdometry() {
        m_poseEstimator.update(m_gyro.getRotation2d().unaryMinus(), getCurrentDistances());
    }

    public Pose2d getPose() {
      return m_poseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }
    public void driveRobotRelative(ChassisSpeeds speeds) {
      MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
      wheelSpeeds.desaturate(.5);  
      setSpeeds(wheelSpeeds);
    }
    public static MecanumDriveWheelPositions getCurrentDistances() {
        return new MecanumDriveWheelPositions( // de rotaciones, convertidos a metros por rueda
            m_frontLeft.getEncoder().getPosition()  * DriveConstants.conversionFactor,
            m_rearLeft.getEncoder().getPosition()   * DriveConstants.conversionFactor,
            m_frontRight.getEncoder().getPosition() * DriveConstants.conversionFactor,
            m_rearRight.getEncoder().getPosition()  * DriveConstants.conversionFactor
          );
    }

    public MecanumDriveWheelSpeeds getCurrentState() { // RPM a RPS a MPS
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

    public double getAverageEncoderDistance() {
        // La posicion del encoder de SparkMax se da en rotaciones.
        double frontLeftRotations = m_frontLeft.getEncoder().getPosition();
        double frontRightRotations = m_frontRight.getEncoder().getPosition();
        double rearLeftRotations = m_rearLeft.getEncoder().getPosition();
        double rearRightRotations = m_rearRight.getEncoder().getPosition();

        // Tomamos el promedio de las rotaciones de las cuatro ruedas
        double averageRotations = (frontLeftRotations + frontRightRotations + rearLeftRotations + rearRightRotations) / 4.0;

        // Convertimos las rotaciones a metros
        return averageRotations * DriveConstants.conversionFactor;
    }

    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        double velocityConversionFactor = 60.0 / DriveConstants.conversionFactor;

        final double frontLeftCurrent = m_frontLeft.getEncoder().getVelocity() / velocityConversionFactor;
        final double frontRightCurrent = m_frontRight.getEncoder().getVelocity() / velocityConversionFactor; 
        final double rearLeftCurrent = m_rearLeft.getEncoder().getVelocity() / velocityConversionFactor;
        final double rearRightCurrent = m_rearRight.getEncoder().getVelocity() / velocityConversionFactor;

        final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
        final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
        final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
        final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

        final double frontLeftOutput = m_frontLeftPIDController.calculate(frontLeftCurrent, speeds.frontLeftMetersPerSecond);
        final double frontRightOutput = m_frontRightPIDController.calculate(frontRightCurrent, speeds.frontRightMetersPerSecond);
        final double backLeftOutput = m_backLeftPIDController.calculate(rearLeftCurrent, speeds.rearLeftMetersPerSecond);
        final double backRightOutput = m_backRightPIDController.calculate(rearRightCurrent, speeds.rearRightMetersPerSecond);

        m_frontLeft.setVoltage(frontLeftOutput + frontLeftFeedforward);
        m_frontRight.setVoltage(frontRightOutput + frontRightFeedforward);
        m_rearLeft.setVoltage(backLeftOutput + backLeftFeedforward);
        m_rearRight.setVoltage(backRightOutput + backRightFeedforward);
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
      return m_kinematics.toChassisSpeeds(getCurrentState());
    }


    public void configureAutoBuilder() {
      try {
        var config = RobotConfig.fromGUISettings();
      //Configure AutoBuilder last
        AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
              new PIDConstants(2, 0, 0),
              new PIDConstants(2, 0, 0)
            ), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
          );
    } catch (Exception e) {
        e.printStackTrace();
      }
    }
//TODO: UNCOMMENT SYSID
  //private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
  //  new SysIdRoutine.Config(),
  //  new SysIdRoutine.Mechanism(
  //      // EXPLICACIÓN: Recibe un objeto 'volts' y le extraemos el valor numérico
  //      (Voltage volts) -> {
  //          double v = volts.in(edu.wpi.first.units.Units.Volts);
  //          m_frontLeft.setVoltage(v);
  //          m_frontRight.setVoltage(v);
  //          m_rearLeft.setVoltage(v);
  //          m_rearRight.setVoltage(v);
  //      },
  //      // EXPLICACIÓN: Guardamos los datos de rendimiento del motor
  //      log -> {
  //          // Log para el lado izquierdo (puedes agruparlos así)
  //          log.motor("drive-left")
  //              .voltage(Volts.of(m_frontLeft.getBusVoltage() * m_frontLeft.getAppliedOutput()))
  //              .linearPosition(Meters.of(m_frontLeft.getEncoder().getPosition() * DriveConstants.conversionFactor))
  //              .linearVelocity(MetersPerSecond.of(m_frontLeft.getEncoder().getVelocity() * (DriveConstants.conversionFactor / 60.0)));
  //          
  //          // Log para el lado derecho
  //          log.motor("drive-right")
  //              .voltage(Volts.of(m_frontRight.getBusVoltage() * m_frontRight.getAppliedOutput()))
  //              .linearPosition(Meters.of(m_frontRight.getEncoder().getPosition() * DriveConstants.conversionFactor))
  //              .linearVelocity(MetersPerSecond.of(m_frontRight.getEncoder().getVelocity() * (DriveConstants.conversionFactor / 60.0)));
  //      },
  //      this
  //  )
  //);
//
  //public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //    return m_sysIdRoutine.quasistatic(direction);
  //}
//
  //public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //    return m_sysIdRoutine.dynamic(direction);
  //}

  @Override
  public void periodic() {
      updateOdometry(); 
      
      SmartDashboard.putNumber("Robot X", getPose().getX());
      SmartDashboard.putNumber("Robot Y", getPose().getY());
  }
}