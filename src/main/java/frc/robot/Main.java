package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * La clase Main es literalmente el punto de entrada (la puerta principal) oficial de toda la aplicación 
 * de la RoboRIO o el simulador local de Java en tu computadora.
 * 
 * NOTA: Esta clase NO se debe tocar JAMÁS. Es auto-generada por el framework abstracto de desarrollo de campo WPILib
 *  y es vital para montar virtualmente la base fundamental e inicialización del núcleo base (Timers, OS, y Comandos).
 */
public final class Main {
  // Constructor fantasma privado que prohíbe las clonaciones letales que rompan todo en otros lugares de memoria (Singleton).
  private Main() {}

  /** 
   * "Main" es lo primero que corre C++. Llama al método fundamental genérico inter-puentes del SO para decirle:
   * "Despierta e Inyecta el constructor de software principal Robot.java a los controladores y empápalo a la electrónica".
   */
  public static void main(String... args) { RobotBase.startRobot(Robot::new); }
}
