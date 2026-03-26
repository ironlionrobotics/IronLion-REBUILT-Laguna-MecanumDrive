package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

public class DriveSubsystem extends SubsystemBase  {
  
    public static final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
    public static final SparkMax m_rearLeft = new SparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
    public static final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
    public static final SparkMax m_rearRight = new SparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

    public static final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroPort);
    
    public static final PIDController m_frontLeftPIDController = new PIDController(.2, 0, 0);
    public static final PIDController m_frontRightPIDController = new PIDController(.2, 0, 0);
    public static final PIDController m_backLeftPIDController = new PIDController(.2, 0, 0);
    public static final PIDController m_backRightPIDController = new PIDController(.2, 0, 0);

    public static final Translation2d m_frontLeftLocation = new Translation2d(0.31, 0.21);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.31, -0.21); 
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.31, 0.21);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.31, -0.21);
    
    private final Field2d m_field = new Field2d();
    
    public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    public static final MecanumDriveKinematics m_kinematics = 
      new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
      );
          
    public edu.wpi.first.math.geometry.Rotation2d getHeading() {
      return m_gyro.getRotation2d(); //todo: unary minus removed 
    }
    
    public final MecanumDrivePoseEstimator m_poseEstimator =
      new MecanumDrivePoseEstimator(
          m_kinematics,
          getHeading(),
          getCurrentDistances(),
          Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );

    public DriveSubsystem() {
        motorConfig(); 
        SmartDashboard.putData("Field", m_field);
        
    }

    public void resetOdometry(Pose2d pose) {
        m_frontLeft.getEncoder().setPosition(0);
        m_rearLeft.getEncoder().setPosition(0);
        m_frontRight.getEncoder().setPosition(0);
        m_rearRight.getEncoder().setPosition(0);
        m_poseEstimator.resetPosition(getHeading(), getCurrentDistances(), pose);
        m_gyro.setYaw(0);
    }
    
    public void updateOdometry() { 
        m_poseEstimator.update(getHeading(), getCurrentDistances());
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public Pose2d getPose() {
      return m_poseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
      double kStrafeMultiplier = 1.2;
      speeds.vyMetersPerSecond = speeds.vyMetersPerSecond * kStrafeMultiplier;
      MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
      wheelSpeeds.desaturate(4.5); 
      setSpeeds(wheelSpeeds);
    }

    public static MecanumDriveWheelPositions getCurrentDistances() {
        return new MecanumDriveWheelPositions( 
            m_frontLeft.getEncoder().getPosition(),
            m_frontRight.getEncoder().getPosition(),
            m_rearLeft.getEncoder().getPosition(),
            m_rearRight.getEncoder().getPosition() 
          );
    }

    public MecanumDriveWheelSpeeds getCurrentState() { 
        return new MecanumDriveWheelSpeeds(
            m_frontLeft.getEncoder().getVelocity(),
            m_frontRight.getEncoder().getVelocity(),
            m_rearLeft.getEncoder().getVelocity(),
            m_rearRight.getEncoder().getVelocity()
        );
    }

    public double getAverageEncoderDistance() {
        double frontLeftRotations = m_frontLeft.getEncoder().getPosition();
        double frontRightRotations = m_frontRight.getEncoder().getPosition();
        double rearLeftRotations = m_rearLeft.getEncoder().getPosition();
        double rearRightRotations = m_rearRight.getEncoder().getPosition();

        return (frontLeftRotations + frontRightRotations + rearLeftRotations + rearRightRotations) / 4.0;
    }

    public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
        final double frontLeftCurrent = m_frontLeft.getEncoder().getVelocity();
        final double frontRightCurrent = m_frontRight.getEncoder().getVelocity(); 
        final double rearLeftCurrent = m_rearLeft.getEncoder().getVelocity();
        final double rearRightCurrent = m_rearRight.getEncoder().getVelocity();

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
        RobotConfig config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
          this::getPose, 
          this::resetPose, 
          this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), 
              new PIDConstants(4.5, 0.0, 0.0)  
            ), 
            config, 
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
    
    public static void motorConfig() {
        SparkMaxConfig commmConfig = new SparkMaxConfig();
        commmConfig.idleMode(SparkBaseConfig.IdleMode.kBrake)
          .smartCurrentLimit(40)
          .encoder
            .positionConversionFactor(DriveConstants.conversionFactor)
            .velocityConversionFactor(DriveConstants.conversionFactor / 60.0); 
        
        SparkBaseConfig leftConfig = new SparkMaxConfig().idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftConfig
          .apply(commmConfig)
          .inverted(false);
        
        SparkBaseConfig rightConfig = new SparkMaxConfig().idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightConfig
          .apply(commmConfig)
          .inverted(true);

        m_frontLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);        
        m_frontRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void updateVisionOdometry() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("LeftCamera");

        if (limelightMeasurement != null && limelightMeasurement.tagCount > 0) {
            double xyStds = 0.5; 
            double degStds = 9999999; 

            if (limelightMeasurement.tagCount >= 2) {
                xyStds = 0.1; 
            } 
            else if (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 2.0) {
                xyStds = 0.3;
            } 
            else {
                xyStds = 1.0; 
            }

            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, degStds));
            m_poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
      }

  @Override
  public void periodic() {
      updateOdometry(); 
      updateVisionOdometry();

      m_field.setRobotPose(getPose());
  }
}