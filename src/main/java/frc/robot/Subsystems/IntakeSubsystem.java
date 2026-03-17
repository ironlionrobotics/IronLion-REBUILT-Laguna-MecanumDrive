package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunableConstants;

/**
 * IntakeSubsystem controla el mecanismo recolector del robot.
 * Se encarga de tragar las piezas del suelo (NeoIntake) y subir o bajar el brazo protector (IntakeElevar).
 */
public class IntakeSubsystem extends SubsystemBase {

    // SparkMax son los controladores inteligentes físicos conectados por cables CAN bus.
    // Usamos el tipo kBrushless porque de la misma forma, los motores NEO y VORTEX no tienen escobillas.
    private final SparkMax m_NeoIntake = new SparkMax(IntakeConstants.kNeoIntakePort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_IntakeElevar = new SparkMax(IntakeConstants.kIntakeElevarPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);   

    /** Constructor: Se llama una sola vez al encender el robot para configurar la electrónica. */
    public IntakeSubsystem() {
      // ------ Configuración del motor recolector (Rodillos) ------
      com.revrobotics.spark.config.SparkMaxConfig intakeConfig = new com.revrobotics.spark.config.SparkMaxConfig();
      // kCoast: Significa que si soltamos el botón, los rodillos giran libremente hasta detenerse suavemente.
      intakeConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
                  .smartCurrentLimit(IntakeConstants.kIntakeCurrentLimit) // Límite para no quemarlo
                  .inverted(true); // Cambiar el + y - interno por software para no recablear
      
      // ------ Configuración del motor elevador (Brazo del Intake) ------
      com.revrobotics.spark.config.SparkMaxConfig elevatorConfig = new com.revrobotics.spark.config.SparkMaxConfig();
      // kBrake: Frena totalmente en seco para que el mecanismo no caiga por gravedad si soltamos el botón.
      elevatorConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                    .smartCurrentLimit(IntakeConstants.kElevatorCurrentLimit)
                    .inverted(false);

      // Inyectar estas configuraciones quemándolas en la memoria flash de los Spaks, garantizando persistencia en caídas de voltaje.
      m_NeoIntake.configure(intakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
      m_IntakeElevar.configure(elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

      // Reiniciar el encoder magnético interno a ceros
      m_IntakeElevar.getEncoder().setPosition(0);
      m_IntakeElevar.setSetpoint(0, ControlType.kPosition);    
    }

    // =========================================================
    // MÉTODOS DIRECTOS (Se ejecutan inmediatamente mediante Software Primitivo)
    // =========================================================
    
    /** Gira los rodillos para recolectar. Velocidad ajustable en vivo desde TunableConstants. */
    public void runIntake() {
        m_NeoIntake.set(TunableConstants.intakeSpeed);
    }

    /** Gira los rodillos al revés. Velocidad ajustable en vivo. */
    public void runIntakeReverse() {
        m_NeoIntake.set(TunableConstants.intakeReverseSpeed);
    }

    /** Escribe Potencia Cero de Voltaje neto a Rodillos */
    public void stopIntake() {
        m_NeoIntake.set(0);
    }

    /** Manda mover mecánicamente el brazo arriba con velocidades lentas proveídas de los Joysticks Análogos */
    public void runIntakeElevar(double speed) {
        m_IntakeElevar.set(speed);
    }

    public void stopIntakeElevar() {
        m_IntakeElevar.set(0);
    }

    // =========================================================
    // FACTORÍA DE COMANDOS (Encapsulamiento Limpio de Tareas WPILib)
    // =========================================================
    // Estos retornan objetos "Command" que el robot.java encola por peticiones y auto-destruye cuando soltamos triggers.
    
    /** Comando "runEnd": Ejecuta runIntake MIENTRAS esté activo. Cuando termine llama a stopIntake por seguridad. */
    public Command runIntakeCommand() {
        return this.runEnd(this::runIntake, this::stopIntake);
    }
    
    /** Comando "runEnd": Expulsor */
    public Command runIntakeReverseCommand() {
        return this.runEnd(this::runIntakeReverse, this::stopIntake);
    }

    /** Comando "runOnce": Dispara acción de paro inmediato sólo un tic de control y descarta este comando. */
    public Command stopIntakeCommand() {
        return this.runOnce(this::stopIntake);
    }

    /** Comando que exige una Velocidad al ser llamado desde la configuración Xbox */
    public Command runIntakeElevarCommand(double speed) {
        return this.runEnd(() -> runIntakeElevar(speed), this::stopIntakeElevar);
    }

    public Command stopIntakeElevarCommand() {
        return this.runOnce(this::stopIntakeElevar);
    }

    // Getters
    public SparkMax getNeoIntake() {
        return m_NeoIntake;
    }

    public SparkMax getIntakeElevar() {
        return m_IntakeElevar;
    }

    @Override
    public void periodic() {
        // Nada requerido en cada loop.
    }
}