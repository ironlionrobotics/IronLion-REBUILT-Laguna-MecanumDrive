package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunableConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_NeoIntake = new SparkMax(DriveConstants.kNeoIntakePort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_IntakeElevar = new SparkMax(DriveConstants.kIntakeArmPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   

    public IntakeSubsystem() { 
      SparkBaseConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
          .idleMode(SparkBaseConfig.IdleMode.kCoast)
          .smartCurrentLimit(30)
          .inverted(true);

      SparkBaseConfig intakeElevarConfig = new SparkMaxConfig();
        intakeElevarConfig
          .idleMode(SparkBaseConfig.IdleMode.kCoast)
          .inverted(false)
          .smartCurrentLimit(30);

      intakeElevarConfig.encoder.positionConversionFactor(15.0); 

      m_NeoIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_IntakeElevar.configure(intakeElevarConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      m_IntakeElevar.getEncoder().setPosition(0);
    }

    public void runIntake() {
        m_NeoIntake.set(TunableConstants.intakeSpeed);
    }

    public void stopIntake() {
        m_NeoIntake.stopMotor();
    }

    public Command runIntakeCommand() {
        return this.run(this::runIntake);
      }

    public Command stopIntakeCommand() {
        return this.run(this::stopIntake);
      }

    public void runIntakeArm() {
        m_IntakeElevar.set(TunableConstants.intakeElevarSpeed);
    }

    public void runIntakeArmReverse() {
        m_IntakeElevar.set(TunableConstants.intakeElevarSpeedReverse);
    }

    public void stopIntakeArm() {
        m_IntakeElevar.stopMotor();
    }

    public Command runIntakeArmCommand() {
        return this.runOnce(this::runIntakeArm);
      }

    public Command runIntakeArmReverseCommand() {
      return this.runOnce(this::runIntakeArmReverse);
    }

    public Command stopIntakeArmCommand() {
        return this.runOnce(this::stopIntakeArm);
      }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Arm Angle", m_IntakeElevar.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Motor Temp (C)", m_IntakeElevar.getMotorTemperature());
    }
}