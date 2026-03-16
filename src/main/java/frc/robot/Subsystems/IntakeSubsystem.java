package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_NeoIntake = new SparkMax(IntakeConstants.kNeoIntakePort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_IntakeElevar = new SparkMax(IntakeConstants.kIntakeElevarPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   

    public IntakeSubsystem() {
      com.revrobotics.spark.config.SparkMaxConfig intakeConfig = new com.revrobotics.spark.config.SparkMaxConfig();
      intakeConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
                  .smartCurrentLimit(IntakeConstants.kIntakeCurrentLimit)
                  .inverted(true);
      
      com.revrobotics.spark.config.SparkMaxConfig elevatorConfig = new com.revrobotics.spark.config.SparkMaxConfig();
      elevatorConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                    .smartCurrentLimit(IntakeConstants.kElevatorCurrentLimit)
                    .inverted(false);

      m_NeoIntake.configure(intakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
      m_IntakeElevar.configure(elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

      m_IntakeElevar.getEncoder().setPosition(0);
      m_IntakeElevar.setSetpoint(0, ControlType.kPosition);    
    }

    public void runIntake() {
        m_NeoIntake.set(IntakeConstants.kIntakeSpeed);
    }

    public void runIntakeReverse() {
        m_NeoIntake.set(IntakeConstants.kIntakeReverseSpeed);
    }

    public void stopIntake() {
        m_NeoIntake.set(0);
    }

    public void runIntakeElevar(double speed) {
        m_IntakeElevar.set(speed);
    }

    public void stopIntakeElevar() {
        m_IntakeElevar.set(0);
    }

    public Command runIntakeCommand() {
        return this.runEnd(this::runIntake, this::stopIntake);
    }
    
    public Command runIntakeReverseCommand() {
        return this.runEnd(this::runIntakeReverse, this::stopIntake);
    }

    public Command stopIntakeCommand() {
        return this.runOnce(this::stopIntake);
    }

    public Command runIntakeElevarCommand(double speed) {
        return this.runEnd(() -> runIntakeElevar(speed), this::stopIntakeElevar);
    }

    public Command stopIntakeElevarCommand() {
        return this.runOnce(this::stopIntakeElevar);
    }

    public SparkMax getNeoIntake() {
        return m_NeoIntake;
    }

    public SparkMax getIntakeElevar() {
        return m_IntakeElevar;
    }

    @Override
    public void periodic() {
        
    }
}