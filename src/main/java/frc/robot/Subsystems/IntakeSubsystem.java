package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SafetyConstants;
import frc.robot.Constants.TunableConstants;

public class IntakeSubsystem extends SubsystemBase {

    public final SparkMax m_NeoIntake = new SparkMax(DriveConstants.kNeoIntakePort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_IntakeElevar = new SparkMax(DriveConstants.kIntakeArmPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   

    public IntakeSubsystem() { 
      SparkBaseConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
          .idleMode(SparkBaseConfig.IdleMode.kCoast)
          .smartCurrentLimit((int) SafetyConstants.kMaxMotorCurrentAmps)
          .inverted(true);

      SparkBaseConfig intakeElevarConfig = new SparkMaxConfig();
        intakeElevarConfig
          .idleMode(SparkBaseConfig.IdleMode.kCoast)
          .inverted(false)
          .smartCurrentLimit((int) SafetyConstants.kMaxMotorCurrentAmps);

      intakeElevarConfig.encoder.positionConversionFactor(1.0 / 15.0); 

      m_NeoIntake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_IntakeElevar.configure(intakeElevarConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      m_IntakeElevar.getEncoder().setPosition(0);
    }

    public void runIntake() {           m_NeoIntake.set(TunableConstants.intakeSpeed); }
    public void runIntakeReverse() {    m_NeoIntake.set(TunableConstants.intakeReverseSpeed); }
    public void stopIntake() {          m_NeoIntake.stopMotor(); }

    public void runIntakeArm() {        m_IntakeElevar.set(TunableConstants.intakeElevarSpeed); }
    public void runIntakeArmReverse() { m_IntakeElevar.set(TunableConstants.intakeElevarSpeedReverse); }
    public void stopIntakeArm() {       m_IntakeElevar.stopMotor(); }

    // Command Factories
    public Command runIntakeCommand() { return              this.runOnce(this::runIntake); }
    public Command runIntakeReverseCommand() { return       this.runOnce(this::runIntakeReverse); }
    public Command stopIntakeCommand() { return             this.runOnce(this::stopIntake); }
    public Command runIntakeArmCommand() { return           this.runOnce(this::runIntakeArm); }
    public Command runIntakeArmReverseCommand() { return    this.runOnce(this::runIntakeArmReverse); }
    public Command stopIntakeArmCommand() { return          this.runOnce(this::stopIntakeArm); }
    public Command runIntakeWithArmCommand() { 
        return Commands.sequence(
            this.runOnce(() -> {
                runIntake();     // intake ON
                runIntakeArm();  // arm UP
            }),
            new WaitCommand(0.2), 
            
            this.runOnce(() -> {
                runIntake();     // intake stays ON
                stopIntakeArm(); // Arm STOPS
            }),
            new WaitCommand(0.3) 
        )
        .repeatedly() 
        .finallyDo(() -> {
            stopIntake();
            stopIntakeArm();
        });
    }
    // Telemetry Getters
    public double getArmPosition() { return m_IntakeElevar.getEncoder().getPosition(); }
    public double getArmTemp() { return m_IntakeElevar.getMotorTemperature(); }
    public double getIntakeTemp() { return m_NeoIntake.getMotorTemperature(); }

    @Override
    public void periodic() {
    }
}